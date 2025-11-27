// 初始化地图
var map = new AMap.Map("container", {
    mapStyle: "amap://styles/macaron",
    zoom: 14,
    center: [116.397428, 39.90923], // 默认北京
});

// 初始化控件
var scale = new AMap.Scale({ visible: false });
var toolBar = new AMap.ToolBar({ visible: false, position: { top: '110px', right: '40px' } });
var controlBar = new AMap.ControlBar({ visible: false, position: { top: '10px', right: '10px' } });
var overView = new AMap.HawkEye({ visible: true });

// 添加控件到地图
map.addControl(scale);
map.addControl(toolBar);
map.addControl(controlBar);
map.addControl(overView);

// 当前经纬度坐标（用于路径规划）
let latestLat = null;
let latestLng = null;

// 控件切换函数
function toggleScale(cb) {
    cb.checked ? scale.show() : scale.hide();
}
function toggleToolBar(cb) {
    cb.checked ? toolBar.show() : toolBar.hide();
}
function toggleControlBar(cb) {
    cb.checked ? controlBar.show() : controlBar.hide();
}
function toggleOverViewShow(cb) {
    cb.checked ? overView.show() : overView.hide();
}

// 页面加载后不自动绘制路径
window.onload = () => {
    document.getElementById("panel").style.display = "none";  // 初始隐藏结果面板
    // 初始化表格
    const table = document.getElementById('data-table');
    allFields.forEach(key => {
        const row = document.createElement('tr');
        row.innerHTML = `<td>${fieldLabels[key] || key}</td><td id="val-${key}">null</td>`;
        table.appendChild(row);
    });

    marker = new AMap.Marker({
        position: [126.6101, 45.7108],
        title: '车辆当前位置',
        icon: '/static/img/icon.png',
        map: map,
        anchor: new AMap.Pixel(17.5, 45),  // 图标锚点：水平居中，垂直底部
        offset: new AMap.Pixel(-17.5, -45)  // 图标显示偏移，保证底部中间位置固定
    });
};

// 实时更新 marker 位置
function updateMarker(lng, lat) {
    const position = [lng, lat];
    marker.setPosition(position);
    // map.setCenter(position); // 可选：让地图始终跟随移动
}

// 轨迹绘制相关变量
var polyline = null;
var startMarker = null;
var endMarker = null;

function drawPath(path) {
    if (!path || path.length === 0) return;

    if (polyline) map.remove(polyline);
    if (startMarker) map.remove(startMarker);
    if (endMarker) map.remove(endMarker);

    polyline = new AMap.Polyline({
        path: path,
        isOutline: true,
        outlineColor: '#ffeeee',
        borderWeight: 2,
        strokeColor: "#3366FF",
        strokeOpacity: 1,
        strokeWeight: 5,
        strokeStyle: "solid",
        strokeDasharray: [10, 5]
    });
    map.add(polyline);

    startMarker = new AMap.Marker({
        position: path[0],
        map: map,
        label: { content: "起点", offset: new AMap.Pixel(10, -20) }
    });

    endMarker = new AMap.Marker({
        position: path[path.length - 1],
        map: map,
        label: { content: "终点", offset: new AMap.Pixel(10, -20) }
    });

    map.setFitView(polyline);
}

function refreshPath() {
    fetch('/get_trajectory')
        .then(res => res.json())
        .then(data => {
            if (data.error) {
                alert("获取失败: " + data.error);
                return;
            }
            drawPath(data.path);
        })
        .catch(err => alert("请求失败: " + err));
}

// -------- 驾车路径规划：用户触发版 --------
var driving = new AMap.Driving({
    map: map,
    panel: "panel"
});

function planRoute() {
    var startName = document.getElementById("start").value;
    var end = document.getElementById("end").value;
    var startCoord = document.getElementById("start-coord").value;

    if ((!startName && !startCoord) || !end) {
        alert("请填写出发地和目的地");
        return;
    }

    var endPoint = { keyword: end };
    // 优先使用start-coord
    if (startCoord) {
        const [lng, lat] = startCoord.split(',').map(parseFloat);
        const geocoder = new AMap.Geocoder();

        geocoder.getAddress([lng, lat], function(status, result) {
            if (status === 'complete' && result.regeocode) {
                const addr = result.regeocode.formattedAddress;
                console.log("解析坐标得到地址：", addr);
                const startPoint = { keyword: addr };

                driving.search([startPoint, endPoint], routeCallback);
            } else {
                alert("起点坐标解析失败，无法获取地址");
                console.error(result);
            }
        });

    } else {
        // 没有坐标，直接用关键词
        const startPoint = { keyword: startName };
        driving.search([startPoint, endPoint], routeCallback);
    }
}

function routeCallback(status, result) {
    if (status === 'complete') {
        console.log('路径规划成功');
        document.getElementById("panel").style.display = "block";
        currentResult = result;
    } else {
        console.error('路径规划失败：', result);
        alert("路径规划失败，请检查地址是否正确");
        document.getElementById("panel").style.display = "none";
        currentResult = null;
    }
}

function startTrip() {
    if (!currentResult) {
        alert("请先规划路径！");
        return;
    }

    fetch('/save_route', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            timestamp: new Date().toISOString(),
            data: currentResult
        })
    }).then(res => res.json())
        .then(resp => {
            if (resp.status === 'ok') {
                const routePath = resp.filename;
                console.log('路径已保存为：' + routePath);

                // 然后自动调用 /extract_steps
                return fetch('/extract_steps', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({
                        route_path: routePath
                    })
                })
                    .then(result => result.json())
                    .then(result => {
                        if (result.status === "ok") {
                            console.log("步骤提取成功：", result.output_file);
                        } else {
                            console.error("提取失败：", result.message);
                        }
                    });

            } else {
                console.log('保存失败');
            }

        });
}

function cancelTrip() {
    // 使用 fetch() 发起 POST 请求
    fetch('/canceltrip', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',  // 请求头
        },
    })
    .then(response => response.json())  // 解析响应为 JSON
    .then(data => {
        if (data.status === "stopped") {
            console.log("行程已取消");
        } else {
            console.log("取消操作失败");
        }
    })
    .catch(error => {
        console.error("请求失败：", error);
    });
}

// MQTT
let mqtt_listener_started = false;
let mqtt_fetchInterval = null;
const allFields = [
    "utc_time", "latitude", "longitude", "num_satellites", "altitude",
    "heading_true", "heading_magnetic", "speed_knots", "speed_kmph",
    "accel_x", "accel_y", "accel_z",
    "gyro_x", "gyro_y", "gyro_z",
    "roll", "pitch", "yaw"
];

const fieldLabels = {
    utc_time: 'UTC时间',
    latitude: '纬度',
    longitude: '经度',
    num_satellites: '卫星数',
    altitude: '海拔',
    heading_true: '真北航向',
    heading_magnetic: '磁北航向',
    speed_knots: '速度（节）',
    speed_kmph: '速度（km/h）',
    accel_x: '加速度X',
    accel_y: '加速度Y',
    accel_z: '加速度Z',
    gyro_x: '角速度X',
    gyro_y: '角速度Y',
    gyro_z: '角速度Z',
    roll: '横滚角',
    pitch: '俯仰角',
    yaw: '偏航角'
};
  


function startMQTT() {
fetch('/start_mqtt', { method: 'POST' })
    .then(res => res.json())
    .then(data => {
    alert("MQTT 状态：" + data.status);
    if (data.status === "started" || data.status === "already running") {
        mqtt_listener_started = true;
        startPolling();  // 启动轮询
    }
    });
}

function startPolling() {
    if (mqtt_fetchInterval !== null) return; // 避免多次启动轮询

    mqtt_fetchInterval = setInterval(() => {
        if (!mqtt_listener_started) return;

        fetch('/latest')
        .then(res => res.ok ? res.json() : null)
        .then(data => {
            if (data) {
            updateTable(data);
            }
        });
    }, 1000); // 每秒请求一次
}

function stopMQTT() {
    mqtt_listener_started = false;
    if (mqtt_fetchInterval) {
      clearInterval(mqtt_fetchInterval);
      mqtt_fetchInterval = null;
    }

    // 发送后端请求关闭 MQTT
    fetch('/stop_mqtt', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
        alert("MQTT 已停止：" + data.status);
        })
        .catch(err => console.error('停止 MQTT 失败', err));
}

function clearStartCoord() {
    document.getElementById('start-coord').value = '';
}

function updateTable(data) {
    for (const key in data) {
        const el = document.getElementById('val-' + key);
        if (el) {
            el.textContent = data[key];
        }
    }

  // 解析坐标并更新地图
    const latStr = data.latitude;
    const lonStr = data.longitude;

    if (latStr && lonStr) {
        const lat = convertToDecimal(latStr);
        const lng = convertToDecimal(lonStr);
        if (lat !== null && lng !== null) {
            latestLat = lat;
            latestLng = lng;
            updateMarker(lng, lat);  // 高德用的是 [lng, lat]
        }
    }
}

function toLabel(key) {
    return fieldLabels[key] || key;
}

function convertToDecimal(coordStr) {
    if (typeof coordStr !== 'string') return null;
    let value = parseFloat(coordStr);
    if (coordStr.includes('S') || coordStr.includes('W')) value = -value;
    return isNaN(value) ? null : value;
}

function useCurrentLocation() {
    if (latestLat !== null && latestLng !== null) {
        document.getElementById('start').value = '当前位置';
        // 设置 hidden input 存储真实坐标供后续路径规划使用
        document.getElementById('start-coord').value = `${latestLng},${latestLat}`;

        // 移动地图到当前位置
        const position = [latestLng, latestLat];
        map.setCenter(position); // 将地图中心设置为当前位置

    } else {
        alert('位置获取失败，请稍后重试');
    }
}