// 全局变量
let scene, camera, renderer;
let controls;
let pointCloud = null;
let boxes = [];
let boxIndex = 0;
let lines = [];
let lineIndex = 0;

const DEFAULT_BOX_CONFIG = {
    x: 34.56,
    y: 0.0,
    z: -3.0,
    w: 69.12,
    l: 79.36,
    h: 4.0,
    rt: 0.0
};

const TRANSFORM_CONFIG_KEY = 'pointCloudTransformConfig';
const LINES_CONFIG_KEY = 'lineSegmentsConfig';

// 点云变换参数
let cloudTransform = {
    roll: 0,
    pitch: 0,
    yaw: 0,
    x: 0,
    y: 0,
    z: 0
};

function getDefaultTransformConfig() {
    return {
        roll: 0,
        pitch: 0,
        yaw: 0,
        x: 0,
        y: 0,
        z: 0
    };
}

function ensureNumericTransform(config) {
    const fallback = getDefaultTransformConfig();
    const result = {};
    Object.keys(fallback).forEach(key => {
        const value = config && typeof config[key] === 'number' ? config[key] : parseFloat(config?.[key]);
        result[key] = isNaN(value) ? fallback[key] : value;
    });
    return result;
}

function ensureNumber(value, fallback = 0) {
    const num = parseFloat(value);
    return Number.isFinite(num) ? num : fallback;
}

function loadTransformConfig() {
    let configData = null;
    try {
        if (typeof window !== 'undefined' && window.localStorage) {
            const stored = window.localStorage.getItem(TRANSFORM_CONFIG_KEY);
            if (stored) {
                configData = JSON.parse(stored);
            }
        }
    } catch (error) {
        console.warn('读取点云变换配置失败，使用默认值', error);
    }

    if (!configData) {
        configData = getDefaultTransformConfig();
        saveTransformConfig(configData);
    }

    const numericConfig = ensureNumericTransform(configData);
    Object.assign(cloudTransform, numericConfig);
}

function saveTransformConfig(config) {
    const dataToSave = ensureNumericTransform(config || cloudTransform);
    try {
        if (typeof window !== 'undefined' && window.localStorage) {
            window.localStorage.setItem(TRANSFORM_CONFIG_KEY, JSON.stringify(dataToSave));
        }
    } catch (error) {
        console.warn('保存点云变换配置失败', error);
    }
}

function saveLinesConfig() {
    if (typeof window === 'undefined' || !window.localStorage) return;

    const dataToSave = lines.map(line => ({
        start: { x: line.start.x, y: line.start.y, z: line.start.z },
        end: { x: line.end.x, y: line.end.y, z: line.end.z },
        color: line.color || 0x00ff00
    }));

    try {
        window.localStorage.setItem(LINES_CONFIG_KEY, JSON.stringify(dataToSave));
    } catch (error) {
        console.warn('保存线段配置失败', error);
    }
}

function sanitizeLineConfigEntry(entry) {
    if (!entry || typeof entry !== 'object') return null;

    const defaultStart = { x: 0, y: 0, z: 0 };
    const defaultEnd = { x: 1, y: 0, z: 0 };

    const sanitizePoint = (point, fallback) => ({
        x: ensureNumber(point?.x, fallback.x),
        y: ensureNumber(point?.y, fallback.y),
        z: ensureNumber(point?.z, fallback.z)
    });

    const start = sanitizePoint(entry.start, defaultStart);
    const end = sanitizePoint(entry.end, defaultEnd);
    const color = typeof entry.color === 'number' && Number.isFinite(entry.color) ? entry.color : 0x00ff00;

    return { start, end, color };
}

function loadLinesConfig() {
    lines.forEach(line => {
        if (line.mesh) {
            scene.remove(line.mesh);
            line.mesh.geometry.dispose();
            line.mesh.material.dispose();
        }
    });
    lines = [];
    lineIndex = 0;

    let storedData = null;
    try {
        if (typeof window !== 'undefined' && window.localStorage) {
            const raw = window.localStorage.getItem(LINES_CONFIG_KEY);
            if (raw) {
                storedData = JSON.parse(raw);
            }
        }
    } catch (error) {
        console.warn('读取线段配置失败，将使用默认值', error);
    }

    if (!Array.isArray(storedData)) {
        storedData = [];
    }

    storedData.forEach(entry => {
        const sanitized = sanitizeLineConfigEntry(entry);
        if (!sanitized) return;

        const newLine = {
            id: lineIndex++,
            start: sanitized.start,
            end: sanitized.end,
            color: sanitized.color,
            mesh: null
        };
        lines.push(newLine);
        updateLine3D(newLine.id);
    });

    renderLinesList();
    saveLinesConfig();
}

// 原始点云数据
let originalPoints = null;
let originalColors = null;
let transformedPointsCache = null;
let groundPlaneResult = null;

// 初始化Three.js场景
function initScene() {
    const canvas = document.getElementById('renderCanvas');
    
    // 创建场景
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);
    
    // 创建相机 - 使用透视投影
    camera = new THREE.PerspectiveCamera(
        75,
        canvas.clientWidth / canvas.clientHeight,
        0.1,
        1000
    );
    camera.position.set(0, 0, 10);
    
    // 创建渲染器
    renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    
    // 添加轨道控制器
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.rotateSpeed = 0.5;  // 降低旋转灵敏度
    controls.panSpeed = 0.8;     // 平移速度
    controls.zoomSpeed = 1.2;    // 缩放速度
    
    // 添加网格辅助线
    const gridHelper = new THREE.GridHelper(20, 20, 0x444444, 0x222222);
    scene.add(gridHelper);
    
    // 添加坐标轴
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);
    
    // 添加光源
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 10, 5);
    scene.add(directionalLight);
    
    // 渲染循环
    animate();
    
    // 窗口大小调整
    window.addEventListener('resize', onWindowResize);
}

// 渲染循环
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// 窗口大小调整
function onWindowResize() {
    const canvas = document.getElementById('renderCanvas');
    camera.aspect = canvas.clientWidth / canvas.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(canvas.clientWidth, canvas.clientHeight);
}

// 解析PCD文件
function parsePCD(text) {
    const lines = text.split('\n');
    const header = {};
    let dataStart = -1;
    
    // 解析头部
    for (let i = 0; i < lines.length; i++) {
        const line = lines[i].trim();
        if (!line) continue;
        
        const parts = line.split(/\s+/);
        const key = parts[0].toUpperCase();
        
        if (key === 'DATA') {
            dataStart = i + 1;
            break;
        } else if (parts.length > 1) {
            header[key] = parts[1];
        }
    }
    
    if (dataStart === -1) {
        throw new Error('Invalid PCD file format');
    }
    
    // 解析点云数据 - 按行计数，每行一个点
    const points = [];
    const colors = [];
    
    for (let i = dataStart; i < lines.length; i++) {
        const line = lines[i].trim();
        if (!line || line.startsWith('#')) continue;
        
        // 支持逗号分隔和空格分隔
        const values = line.split(/[,\s]+/).map(parseFloat);
        if (values.length >= 3) {
            // 取前四个值: x, y, z, 反射率
            points.push(values[0], values[1], values[2]);
            // 如果有强度信息，转换为颜色
            if (values.length >= 4) {
                const intensity = values[3];
                colors.push(intensity, intensity, intensity);
            } else {
                colors.push(1.0, 1.0, 1.0);
            }
        }
    }
    
    return { points, colors };
}

// 加载点云
function loadPointCloud(file) {
    const reader = new FileReader();
    
    reader.onload = function(e) {
        try {
            const text = e.target.result;
            const { points, colors } = parsePCD(text);
            
            // 保存原始数据
            originalPoints = points;
            originalColors = colors;
            
            // 应用变换
            applyPointCloudTransform();
            
            // 自动调整视角
            fitCameraToPointCloud(points);
            
            // 显示点云数量
            const pointCount = points.length / 3;
            document.getElementById('pointCloudInfo').style.display = 'block';
            document.getElementById('pointCount').textContent = pointCount.toLocaleString();
            
            console.log(`点云加载成功: ${pointCount} 个点`);
        } catch (error) {
            alert('点云文件解析失败: ' + error.message);
            console.error(error);
        }
    };
    
    reader.readAsText(file);
}

// 变换点云 (参考GeneralParser的TransformPoint函数)
function transformPoint(x, y, z, transform) {
    const cosa = Math.cos(transform.roll);
    const sina = Math.sin(transform.roll);
    const cosb = Math.cos(transform.pitch);
    const sinb = Math.sin(transform.pitch);
    const cosc = Math.cos(transform.yaw);
    const sinc = Math.sin(transform.yaw);
    
    const x_ = cosb * cosc * x + (sina * sinb * cosc - cosa * sinc) * y +
                (sina * sinc + cosa * sinb * cosc) * z + transform.x;
    const y_ = cosb * sinc * x + (cosa * cosc + sina * sinb * sinc) * y +
                (cosa * sinb * sinc - sina * cosc) * z + transform.y;
    const z_ = -sinb * x + sina * cosb * y + cosa * cosb * z + transform.z;
    
    return {x: x_, y: y_, z: z_};
}

// 应用点云变换
function applyPointCloudTransform() {
    if (!originalPoints) return;
    
    // 清除旧的点云
    if (pointCloud) {
        scene.remove(pointCloud);
        pointCloud.geometry.dispose();
    }
    
    // 应用变换到点云
    const transformedPoints = [...originalPoints];
    for (let i = 0; i < transformedPoints.length; i += 3) {
        const transformed = transformPoint(
            transformedPoints[i],
            transformedPoints[i + 1],
            transformedPoints[i + 2],
            cloudTransform
        );
        transformedPoints[i] = transformed.x;
        transformedPoints[i + 1] = transformed.y;
        transformedPoints[i + 2] = transformed.z;
    }
    
    transformedPointsCache = transformedPoints.slice();
    groundPlaneResult = null;
    updateGroundPlaneDisplay(null, '点云已更新，请重新计算地面平面');

    // 创建新点云
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(transformedPoints, 3));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(originalColors, 3));
    
    const material = new THREE.PointsMaterial({
        size: 0.01,
        vertexColors: true,
        sizeAttenuation: false
    });
    
    pointCloud = new THREE.Points(geometry, material);
    scene.add(pointCloud);
}

// 更新点云变换参数
function updateCloudTransform(param, delta) {
    cloudTransform[param] += delta;
    applyPointCloudTransform();
    saveTransformConfig();
    renderCloudTransformControls();
}

// 直接从输入框更新点云变换
function updateCloudTransformFromInput(param, value, isAngle) {
    const numValue = parseFloat(value);
    if (!isNaN(numValue)) {
        // 如果是角度参数，需要从度转换为弧度
        cloudTransform[param] = isAngle ? numValue * Math.PI / 180 : numValue;
        applyPointCloudTransform();
        saveTransformConfig();
    }
}

// 自动调整相机视角
function fitCameraToPointCloud(points) {
    if (points.length === 0) return;
    
    // 计算点云边界
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;
    
    for (let i = 0; i < points.length; i += 3) {
        minX = Math.min(minX, points[i]);
        maxX = Math.max(maxX, points[i]);
        minY = Math.min(minY, points[i + 1]);
        maxY = Math.max(maxY, points[i + 1]);
        minZ = Math.min(minZ, points[i + 2]);
        maxZ = Math.max(maxZ, points[i + 2]);
    }
    
    // 计算中心点和大小
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const centerZ = (minZ + maxZ) / 2;
    
    const sizeX = maxX - minX;
    const sizeY = maxY - minY;
    const sizeZ = maxZ - minZ;
    const maxSize = Math.max(sizeX, sizeY, sizeZ);
    
    // 调整相机位置
    camera.position.set(centerX + maxSize, centerY + maxSize, centerZ + maxSize);
    camera.lookAt(centerX, centerY, centerZ);
    camera.updateProjectionMatrix();
    
    // 调整控制器目标
    controls.target.set(centerX, centerY, centerZ);
    controls.update();
}

// 计算3D框体的8个角点
function boxCorners(box) {
    const cx = box.x, cy = box.y, cz = box.z;
    const w = box.w, l = box.l, h = box.h;
    const yaw = box.rt;
    const hw = w * 0.5, hl = l * 0.5, hh = h * 0.5;
    const cosr = Math.cos(yaw), sinr = Math.sin(yaw);
    
    const rot = (x, y) => ({
        x: x * cosr - y * sinr,
        y: x * sinr + y * cosr
    });
    
    const p0 = rot(-hw, -hl);
    const p1 = rot(hw, -hl);
    const p2 = rot(hw, hl);
    const p3 = rot(-hw, hl);
    
    const z0 = cz - hh;
    const z1 = cz + hh;
    
    return [
        {x: cx + p0.x, y: cy + p0.y, z: z0},
        {x: cx + p1.x, y: cy + p1.y, z: z0},
        {x: cx + p2.x, y: cy + p2.y, z: z0},
        {x: cx + p3.x, y: cy + p3.y, z: z0},
        {x: cx + p0.x, y: cy + p0.y, z: z1},
        {x: cx + p1.x, y: cy + p1.y, z: z1},
        {x: cx + p2.x, y: cy + p2.y, z: z1},
        {x: cx + p3.x, y: cy + p3.y, z: z1}
    ];
}

// 创建框体3D对象
function createBox3D(box) {
    const corners = boxCorners(box);
    
    // 定义12条边（参考draw_meshlab.hpp中的edges定义）
    const edges = [
        [0,1], [1,2], [2,3], [3,0],  // 底面
        [4,5], [5,6], [6,7], [7,4],  // 顶面
        [0,4], [1,5], [2,6], [3,7]   // 垂直边
    ];
    
    // 创建线段几何体
    const points = [];
    for (const [i, j] of edges) {
        points.push(
            corners[i].x, corners[i].y, corners[i].z,
            corners[j].x, corners[j].y, corners[j].z
        );
    }
    
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
    
    const material = new THREE.LineBasicMaterial({
        color: box.color || 0xff0000,
        linewidth: 2
    });
    
    return new THREE.LineSegments(geometry, material);
}

// 添加新框体
function addNewBox() {
    const newBox = {
        id: boxIndex++,
        x: 32.1,
        y: 4.5,
        z: -4.3,
        w: 18.0,
        l: 40.0,
        h: 10.0,
        rt: 0.0, // rotation angle
        color: 0xff0000,
        mesh: null
    };
    
    boxes.push(newBox);
    renderBoxesList();
    updateBox3D(newBox.id);
}

// 删除框体
function deleteBox(id) {
    const index = boxes.findIndex(b => b.id === id);
    if (index !== -1) {
        const box = boxes[index];
        if (box.immutable) {
            alert('默认框体不可删除');
            return;
        }
        if (box.mesh) {
            scene.remove(box.mesh);
            box.mesh.geometry.dispose();
            box.mesh.material.dispose();
        }
        boxes.splice(index, 1);
        renderBoxesList();
    }
}

// 更新框体3D显示
function updateBox3D(id) {
    const box = boxes.find(b => b.id === id);
    if (!box) return;
    
    // 移除旧的mesh
    if (box.mesh) {
        scene.remove(box.mesh);
        box.mesh.geometry.dispose();
        box.mesh.material.dispose();
    }
    
    // 创建新的mesh
    box.mesh = createBox3D(box);
    scene.add(box.mesh);
}

// 更新框体参数
function updateBox(id, param, delta) {
    const box = boxes.find(b => b.id === id);
    if (!box) return;
    if (box.immutable) {
        alert('默认框体不可修改');
        return;
    }
    
    if (param === 'w' || param === 'l' || param === 'h') {
        box[param] = Math.max(0.1, box[param] + delta);
    } else {
        box[param] += delta;
    }
    
    updateBox3D(id);
    renderBoxesList();
}

// 直接从输入框更新
function updateBoxFromInput(id, param, value) {
    const box = boxes.find(b => b.id === id);
    if (!box) return;
    if (box.immutable) {
        alert('默认框体不可修改');
        return;
    }
    
    const numValue = parseFloat(value);
    if (!isNaN(numValue)) {
        if (param === 'w' || param === 'l' || param === 'h') {
            box[param] = Math.max(0.1, numValue);
        } else {
            box[param] = numValue;
        }
        updateBox3D(id);
    }
}

// 渲染点云变换控制器
function renderCloudTransformControls() {
    const container = document.getElementById('cloudTransformControls');
    if (!container) return;
    
    const params = [
        {key: 'roll', label: 'Roll'},
        {key: 'pitch', label: 'Pitch'},
        {key: 'yaw', label: 'Yaw'},
        {key: 'x', label: 'X'},
        {key: 'y', label: 'Y'},
        {key: 'z', label: 'Z'}
    ];
    
    let paramsHtml = '';
    params.forEach(param => {
        // 角度参数使用度，其他使用米
        const isAngle = (param.key === 'roll' || param.key === 'pitch' || param.key === 'yaw');
        const step = isAngle ? 0.1 * Math.PI / 180 : 0.1;
        const displayValue = isAngle ? (cloudTransform[param.key] * 180 / Math.PI).toFixed(3) : cloudTransform[param.key].toFixed(3);
        const unitLabel = isAngle ? ' (度)' : '';
        
        paramsHtml += `
            <div class="param-group">
                <span class="param-label">${param.label}:</span>
                <input type="number" class="param-input" value="${displayValue}" 
                       step="${step}" onchange="updateCloudTransformFromInput('${param.key}', this.value, ${isAngle})">
                <button class="btn btn-sm btn-primary" onclick="updateCloudTransform('${param.key}', ${step})">+</button>
                <button class="btn btn-sm btn-primary" onclick="updateCloudTransform('${param.key}', -${step})">-</button>
            </div>
        `;
    });
    
    container.innerHTML = paramsHtml;
}

// 渲染框体列表
function renderBoxesList() {
    const container = document.getElementById('boxesList');
    container.innerHTML = '';
    
    boxes.forEach((box, index) => {
        const boxItem = document.createElement('div');
        boxItem.className = 'box-item';
        
        const params = [
            {key: 'x', label: 'X'},
            {key: 'y', label: 'Y'},
            {key: 'z', label: 'Z'},
            {key: 'w', label: '宽'},
            {key: 'l', label: '长'},
            {key: 'h', label: '高'},
            {key: 'rt', label: '角度'}
        ];
        
        let paramsHtml = '';
        const isImmutable = !!box.immutable;
        params.forEach(param => {
            const disabledAttr = isImmutable ? 'disabled' : '';
            const buttonsHtml = isImmutable ? '' : `
                    <button class="btn btn-sm btn-primary" onclick="updateBox(${box.id}, '${param.key}', 0.1)">+</button>
                    <button class="btn btn-sm btn-primary" onclick="updateBox(${box.id}, '${param.key}', -0.1)">-</button>
                `;
            paramsHtml += `
                <div class="param-group">
                    <span class="param-label">${param.label}:</span>
                    <input type="number" class="param-input" value="${box[param.key].toFixed(3)}" 
                           step="0.1" onchange="updateBoxFromInput(${box.id}, '${param.key}', this.value)" ${disabledAttr}>
                    ${buttonsHtml}
                </div>
            `;
        });
        
        const headerAction = isImmutable
            ? '<span class="param-label" style="color: #a0a0a0;">默认框体</span>'
            : `<button class="btn btn-sm btn-danger" onclick="deleteBox(${box.id})">删除</button>`;

        boxItem.innerHTML = `
            <div class="box-header">
                <span class="box-title">框体 ${index + 1}</span>
                ${headerAction}
            </div>
            ${paramsHtml}
        `;
        
        container.appendChild(boxItem);
    });
}

// 创建线段3D对象
function createLine3D(line) {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute([
        line.start.x, line.start.y, line.start.z,
        line.end.x, line.end.y, line.end.z
    ], 3));

    const material = new THREE.LineBasicMaterial({
        color: line.color || 0x00ff00,
        linewidth: 2
    });

    return new THREE.Line(geometry, material);
}

// 更新线段3D显示
function updateLine3D(id) {
    const line = lines.find(l => l.id === id);
    if (!line) return;

    if (line.mesh) {
        scene.remove(line.mesh);
        line.mesh.geometry.dispose();
        line.mesh.material.dispose();
    }

    line.mesh = createLine3D(line);
    scene.add(line.mesh);
}

// 添加新线段
function addNewLine() {
    const newLine = {
        id: lineIndex++,
        start: { x: 0, y: 0, z: 0 },
        end: { x: 1, y: 0, z: 0 },
        color: 0x00ff00,
        mesh: null
    };

    lines.push(newLine);
    updateLine3D(newLine.id);
    renderLinesList();
    saveLinesConfig();
}

// 删除线段
function deleteLine(id) {
    const index = lines.findIndex(l => l.id === id);
    if (index === -1) return;

    const line = lines[index];
    if (line.mesh) {
        scene.remove(line.mesh);
        line.mesh.geometry.dispose();
        line.mesh.material.dispose();
    }

    lines.splice(index, 1);
    renderLinesList();
    saveLinesConfig();
}

// 更新线段参数
function updateLine(id, pointKey, coordKey, delta) {
    const line = lines.find(l => l.id === id);
    if (!line) return;

    line[pointKey][coordKey] += delta;
    updateLine3D(id);
    renderLinesList();
    saveLinesConfig();
}

// 输入框更新线段参数
function updateLineFromInput(id, pointKey, coordKey, value) {
    const line = lines.find(l => l.id === id);
    if (!line) return;

    const numValue = parseFloat(value);
    if (isNaN(numValue)) return;

    line[pointKey][coordKey] = numValue;
    updateLine3D(id);
    saveLinesConfig();
}

// 渲染线段列表
function renderLinesList() {
    const container = document.getElementById('linesList');
    if (!container) return;

    container.innerHTML = '';

    lines.forEach((line, index) => {
        const lineItem = document.createElement('div');
        lineItem.className = 'box-item';

        const coordinates = ['x', 'y', 'z'];
        const createParamHtml = (pointKey, pointLabel) => {
            return coordinates.map(coord => `
                <div class="param-group">
                    <span class="param-label">${pointLabel}${coord.toUpperCase()}:</span>
                    <input type="number" class="param-input" value="${line[pointKey][coord].toFixed(3)}"
                           step="0.1" onchange="updateLineFromInput(${line.id}, '${pointKey}', '${coord}', this.value)">
                    <button class="btn btn-sm btn-primary" onclick="updateLine(${line.id}, '${pointKey}', '${coord}', 0.1)">+</button>
                    <button class="btn btn-sm btn-primary" onclick="updateLine(${line.id}, '${pointKey}', '${coord}', -0.1)">-</button>
                </div>
            `).join('');
        };

        lineItem.innerHTML = `
            <div class="box-header">
                <span class="box-title">线段 ${index + 1}</span>
                <button class="btn btn-sm btn-danger" onclick="deleteLine(${line.id})">删除</button>
            </div>
            ${createParamHtml('start', '起点')}
            ${createParamHtml('end', '终点')}
        `;

        container.appendChild(lineItem);
    });
}

function initializeDefaultBox() {
    const defaultBox = {
        id: boxIndex++,
        x: DEFAULT_BOX_CONFIG.x,
        y: DEFAULT_BOX_CONFIG.y,
        z: DEFAULT_BOX_CONFIG.z,
        w: DEFAULT_BOX_CONFIG.w,
        l: DEFAULT_BOX_CONFIG.l,
        h: DEFAULT_BOX_CONFIG.h,
        rt: DEFAULT_BOX_CONFIG.rt,
        color: 0xff0000,
        mesh: null,
        immutable: true
    };

    boxes.push(defaultBox);
    updateBox3D(defaultBox.id);
    renderBoxesList();
}

// 导出框体到TXT文件
function exportBoxes() {
    if (boxes.length === 0) {
        alert('没有框体可导出');
        return;
    }
    
    // 添加点云变换参数到导出内容
    // 第一行：计算后的值（弧度）
    const transformLine1 = `点云变换: roll=${cloudTransform.roll.toFixed(6)} pitch=${cloudTransform.pitch.toFixed(6)} yaw=${cloudTransform.yaw.toFixed(6)} x=${cloudTransform.x.toFixed(3)} y=${cloudTransform.y.toFixed(3)} z=${cloudTransform.z.toFixed(3)}`;
    
    // 第二行：公式形式（度转弧度）
    const rollDeg = cloudTransform.roll * 180 / Math.PI;
    const pitchDeg = cloudTransform.pitch * 180 / Math.PI;
    const yawDeg = cloudTransform.yaw * 180 / Math.PI;
    const transformLine2 = `点云变换: roll=${rollDeg.toFixed(2)}*pi/180 pitch=${pitchDeg.toFixed(2)}*pi/180 yaw=${yawDeg.toFixed(2)}*pi/180 x=${cloudTransform.x.toFixed(3)} y=${cloudTransform.y.toFixed(3)} z=${cloudTransform.z.toFixed(3)}`;
    
    const boxLines = boxes.map((box, index) => {
        return `框体${index + 1}: x=${box.x.toFixed(3)} y=${box.y.toFixed(3)} z=${box.z.toFixed(3)} w=${box.w.toFixed(3)} l=${box.l.toFixed(3)} h=${box.h.toFixed(3)} rt=${box.rt.toFixed(3)}`;
    });
    
    const content = [transformLine1, transformLine2, ...boxLines].join('\n');
    const blob = new Blob([content], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `boxes_${Date.now()}.txt`;
    a.click();
    URL.revokeObjectURL(url);
}

function segmentPlaneRANSAC(points, distanceThreshold = 0.05, numIterations = 10000) {
    if (!points || points.length < 9) {
        return null;
    }

    const numPoints = Math.floor(points.length / 3);
    if (numPoints < 3) {
        return null;
    }

    let bestInliers = 0;
    let bestPlane = null;

    const pickIndex = () => Math.floor(Math.random() * numPoints);

    for (let iter = 0; iter < numIterations; iter++) {
        let idx1 = pickIndex();
        let idx2 = pickIndex();
        let idx3 = pickIndex();

        while (idx2 === idx1) idx2 = pickIndex();
        while (idx3 === idx1 || idx3 === idx2) idx3 = pickIndex();

        const p1 = idx1 * 3;
        const p2 = idx2 * 3;
        const p3 = idx3 * 3;

        const p1x = points[p1];
        const p1y = points[p1 + 1];
        const p1z = points[p1 + 2];

        const v1x = points[p2] - p1x;
        const v1y = points[p2 + 1] - p1y;
        const v1z = points[p2 + 2] - p1z;

        const v2x = points[p3] - p1x;
        const v2y = points[p3 + 1] - p1y;
        const v2z = points[p3 + 2] - p1z;

        const nx = v1y * v2z - v1z * v2y;
        const ny = v1z * v2x - v1x * v2z;
        const nz = v1x * v2y - v1y * v2x;

        const norm = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-6) {
            continue;
        }

        const invNorm = 1.0 / norm;
        const nnx = nx * invNorm;
        const nny = ny * invNorm;
        const nnz = nz * invNorm;
        const d = -(nnx * p1x + nny * p1y + nnz * p1z);

        let inliers = 0;
        for (let i = 0; i < numPoints; i++) {
            const base = i * 3;
            const dist = Math.abs(
                nnx * points[base] +
                nny * points[base + 1] +
                nnz * points[base + 2] +
                d
            );
            if (dist <= distanceThreshold) {
                inliers++;
            }
        }

        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestPlane = {
                a: nnx,
                b: nny,
                c: nnz,
                d: d,
                inliers: inliers,
                total: numPoints
            };
        }
    }

    if (!bestPlane) {
        return null;
    }

    if (bestPlane.b < 0) {
        bestPlane.a = -bestPlane.a;
        bestPlane.b = -bestPlane.b;
        bestPlane.c = -bestPlane.c;
        bestPlane.d = -bestPlane.d;
    }

    return bestPlane;
}

function updateGroundPlaneDisplay(result, message) {
    const container = document.getElementById('groundPlaneResult');
    if (!container) return;

    if (message) {
        container.textContent = message;
        return;
    }

    if (!result) {
        container.textContent = '尚未计算地面平面';
        return;
    }

    const { a, b, c, d, inliers, total, durationMs } = result;
    const durationText = durationMs !== undefined ? `<br>耗时: ${durationMs.toFixed(0)} ms` : '';
    container.innerHTML =
        `平面方程: ${a.toFixed(6)}x + ${b.toFixed(6)}y + ${c.toFixed(6)}z + ${d.toFixed(6)} = 0` +
        `<br>内点数量: ${inliers} / ${total}${durationText}`;
}

function calculateGroundPlane() {
    if (!transformedPointsCache || transformedPointsCache.length < 9) {
        alert('请先加载点云并确保点云有效');
        return;
    }

    updateGroundPlaneDisplay(null, '计算中，请稍候...');

    setTimeout(() => {
        const start = performance.now();
        const result = segmentPlaneRANSAC(transformedPointsCache);
        if (!result) {
            updateGroundPlaneDisplay(null, '计算失败或点云数据不足');
            return;
        }
        const duration = performance.now() - start;
        groundPlaneResult = { ...result, durationMs: duration };
        updateGroundPlaneDisplay(groundPlaneResult);
    }, 10);
}

// 保存点云和框体到PCD文件
function saveAsPCD() {
    if (!originalPoints) {
        alert('请先加载点云文件');
        return;
    }
    
    // 生成框体边缘点
    const edgePoints = [];
    const edgeStep = 0.05;
    
    boxes.forEach(box => {
        const corners = boxCorners(box);
        const edges = [
            [0,1],[1,2],[2,3],[3,0],  // 底面
            [4,5],[5,6],[6,7],[7,4],  // 顶面
            [0,4],[1,5],[2,6],[3,7]   // 垂直边
        ];
        
        edges.forEach(([i, j]) => {
            const a = corners[i];
            const b = corners[j];
            const dx = b.x - a.x;
            const dy = b.y - a.y;
            const dz = b.z - a.z;
            const len = Math.sqrt(dx*dx + dy*dy + dz*dz);
            
            if (len <= 1e-6) {
                edgePoints.push([a.x, a.y, a.z, 255]);
            } else {
                const num = Math.max(2, Math.floor(len / edgeStep) + 1);
                for (let k = 0; k < num; k++) {
                    const t = k / (num - 1);
                    edgePoints.push([
                        a.x + t * dx,
                        a.y + t * dy,
                        a.z + t * dz,
                        255
                    ]);
                }
            }
        });
    });
    
    // 创建PCD文件内容
    const totalPoints = originalPoints.length / 3 + edgePoints.length;
    let content = '# .PCD v.7 - Point Cloud Data file format\n';
    content += 'VERSION .7\n';
    content += 'FIELDS x y z intensity\n';
    content += 'SIZE 4 4 4 4\n';
    content += 'TYPE F F F F\n';
    content += 'COUNT 1 1 1 1\n';
    content += `WIDTH ${totalPoints}\n`;
    content += 'HEIGHT 1\n';
    content += 'VIEWPOINT 0 0 0 1 0 0 0\n';
    content += `POINTS ${totalPoints}\n`;
    content += 'DATA ascii\n';
    
    // 添加变换后的点云数据
    for (let i = 0; i < originalPoints.length; i += 3) {
        const transformed = transformPoint(
            originalPoints[i],
            originalPoints[i + 1],
            originalPoints[i + 2],
            cloudTransform
        );
        const intensity = originalColors ? originalColors[i] : 0;
        content += `${transformed.x}, ${transformed.y}, ${transformed.z}, ${intensity}\n`;
    }
    
    // 添加框体边缘点
    edgePoints.forEach(([x, y, z, intensity]) => {
        content += `${x}, ${y}, ${z}, ${intensity}\n`;
    });
    
    // 下载文件
    const blob = new Blob([content], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `pointcloud_with_boxes_${Date.now()}.pcd`;
    a.click();
    URL.revokeObjectURL(url);
    
    console.log(`已保存PCD文件，包含 ${totalPoints} 个点`);
}

// 显示导入对话框
function showImportDialog() {
    document.getElementById('importModal').style.display = 'block';
}

// 关闭导入对话框
function closeImportDialog() {
    document.getElementById('importModal').style.display = 'none';
    document.getElementById('importData').value = '';
}

// 批量导入框体
function importBoxes() {
    const importData = document.getElementById('importData').value.trim();
    if (!importData) {
        alert('请输入框体数据');
        return;
    }
    
    const lines = importData.split('\n');
    let successCount = 0;
    let failCount = 0;
    
    for (const line of lines) {
        const trimmedLine = line.trim();
        if (!trimmedLine) continue;
        
        // 解析每行的数值
        const values = trimmedLine.split(/\s+/);
        if (values.length < 7) {
            console.warn('数据格式不正确:', trimmedLine);
            failCount++;
            continue;
        }
        
        const x = parseFloat(values[0]);
        const y = parseFloat(values[1]);
        const z = parseFloat(values[2]);
        const w = parseFloat(values[3]);
        const l = parseFloat(values[4]);
        const h = parseFloat(values[5]);
        const rt = parseFloat(values[6]);
        
        if (isNaN(x) || isNaN(y) || isNaN(z) || isNaN(w) || isNaN(l) || isNaN(h) || isNaN(rt)) {
            console.warn('包含无效数值:', trimmedLine);
            failCount++;
            continue;
        }
        
        // 创建新框体
        const newBox = {
            id: boxIndex++,
            x: x,
            y: y,
            z: z,
            w: Math.max(0.1, w),
            l: Math.max(0.1, l),
            h: Math.max(0.1, h),
            rt: rt,
            color: 0xff0000,
            mesh: null
        };
        
        boxes.push(newBox);
        updateBox3D(newBox.id);
        successCount++;
    }
    
    if (successCount > 0) {
        renderBoxesList();
        closeImportDialog();
        alert(`成功导入 ${successCount} 个框体${failCount > 0 ? `，${failCount} 个失败` : ''}`);
    } else {
        alert(`导入失败，请检查数据格式`);
    }
}

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    loadTransformConfig();
    initScene();
    loadLinesConfig();
    
    // 初始化点云变换控制器
    renderCloudTransformControls();

    // 添加默认框体
    initializeDefaultBox();
    
    // 文件上传事件
    document.getElementById('pcdFileInput').addEventListener('change', function(e) {
        const file = e.target.files[0];
        if (file) {
            loadPointCloud(file);
        }
    });
    
    // 点击对话框外部关闭
    document.getElementById('importModal').addEventListener('click', function(e) {
        if (e.target === this) {
            closeImportDialog();
        }
    });
});

