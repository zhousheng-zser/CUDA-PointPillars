// 简化的OrbitControls实现
(function(global, factory) {
    typeof exports === 'object' && typeof module !== 'undefined' ? factory(exports) :
    typeof define === 'function' && define.amd ? define(['exports'], factory) :
    (global = global || self, factory(global.THREE = global.THREE || {}));
}(this, (function (exports) { 'use strict';

    var _changeEvent = { type: 'change' };
    var _startEvent = { type: 'start' };
    var _endEvent = { type: 'end' };

    class OrbitControls {
        constructor(object, domElement) {
            this.object = object;
            this.domElement = domElement;
            
            this.enabled = true;
            this.target = new exports.Vector3();
            this.minDistance = 0;
            this.maxDistance = Infinity;
            this.enableDamping = false;
            this.dampingFactor = 0.05;
            
            this.rotateSpeed = 1.0;
            this.zoomSpeed = 1.0;
            this.panSpeed = 1.0;
            
            this.spherical = new exports.Spherical();
            this.sphericalDelta = new exports.Spherical();
            
            this.scale = 1;
            this.panOffset = new exports.Vector3();
            this.offset = new exports.Vector3();
            
            this.rotateStart = new exports.Vector2();
            this.rotateEnd = new exports.Vector2();
            this.rotateDelta = new exports.Vector2();
            
            this.panStart = new exports.Vector2();
            this.panEnd = new exports.Vector2();
            this.panDelta = new exports.Vector2();
            
            this.dollyStart = new exports.Vector2();
            this.dollyEnd = new exports.Vector2();
            this.dollyDelta = new exports.Vector2();
            
            this._state = 0;
            this._panLeft = new exports.Vector3();
            this._panRight = new exports.Vector3();
            this._panUp = new exports.Vector3();
            this._panDown = new exports.Vector3();
            
            this._v = new exports.Vector3();
            
            this.onMouseDown = this.onMouseDown.bind(this);
            this.onMouseMove = this.onMouseMove.bind(this);
            this.onMouseUp = this.onMouseUp.bind(this);
            this.onMouseWheel = this.onMouseWheel.bind(this);
            this.onContextMenu = this.onContextMenu.bind(this);
            this.onPointerDown = this.onPointerDown.bind(this);
            this.onPointerMove = this.onPointerMove.bind(this);
            this.onPointerUp = this.onPointerUp.bind(this);
            
            this.domElement.addEventListener('contextmenu', this.onContextMenu);
            this.domElement.addEventListener('mousedown', this.onMouseDown);
            this.domElement.addEventListener('wheel', this.onMouseWheel);
            
            this.update();
        }
        
        getPolarAngle() {
            return this.spherical.phi;
        }
        
        getAzimuthalAngle() {
            return this.spherical.theta;
        }
        
        getDistance() {
            return this.object.position.distanceTo(this.target);
        }
        
        listenToKeyEvents(domElement) {
            domElement.addEventListener('keydown', this.onKeyDown);
        }
        
        stopListenToKeyEvents() {
            if (this.domElement) {
                this.domElement.removeEventListener('keydown', this.onKeyDown);
            }
        }
        
        saveState() {
            this.target0.copy(this.target);
            this.position0.copy(this.object.position);
            this.zoom0 = this.object.zoom;
        }
        
        reset() {
            this.target.copy(this.target0);
            this.object.position.copy(this.position0);
            this.object.zoom = this.zoom0;
            this.object.updateProjectionMatrix();
            this.dispatchEvent(_changeEvent);
            this.update();
            this._state = 0;
        }
        
        update() {
            const offset = this._v;
            offset.copy(this.object.position).sub(this.target);
            
            this.spherical.setFromVector3(offset);
            if (this.enableDamping) {
                this.spherical.theta += this.sphericalDelta.theta * (1 - this.dampingFactor);
                this.spherical.phi += this.sphericalDelta.phi * (1 - this.dampingFactor);
            } else {
                this.spherical.theta += this.sphericalDelta.theta;
                this.spherical.phi += this.sphericalDelta.phi;
            }
            
            if (this.spherical.phi < 0) this.spherical.phi = 0.00000001;
            if (this.spherical.phi > Math.PI) this.spherical.phi = Math.PI - 0.00000001;
            
            this.spherical.makeSafe();
            this.spherical.radius *= this.scale;
            if (this.spherical.radius < this.minDistance) this.spherical.radius = this.minDistance;
            if (this.spherical.radius > this.maxDistance) this.spherical.radius = this.maxDistance;
            
            this.target.addScaledVector(this._panLeft, -this.panOffset.x);
            this.target.addScaledVector(this._panUp, this.panOffset.y);
            
            offset.setFromSpherical(this.spherical);
            this.object.position.copy(this.target).add(offset);
            this.object.lookAt(this.target);
            
            if (this.enableDamping) {
                this.sphericalDelta.theta *= (1 - this.dampingFactor);
                this.sphericalDelta.phi *= (1 - this.dampingFactor);
                this.panOffset.multiplyScalar(1 - this.dampingFactor);
            } else {
                this.sphericalDelta.set(0, 0, 0);
                this.panOffset.set(0, 0, 0);
            }
            
            this.scale = 1;
        }
        
        onMouseDown(event) {
            if (!this.enabled) return;
            event.preventDefault();
            
            switch (event.button) {
                case 0: this._state = 1; break;
                case 2: this._state = 2; break;
            }
            
            const clientX = event.touches ? event.touches[0].clientX : event.clientX;
            const clientY = event.touches ? event.touches[0].clientY : event.clientY;
            
            if (this._state === 1) {
                this.rotateStart.set(clientX, clientY);
            } else if (this._state === 2) {
                this.panStart.set(clientX, clientY);
            }
            
            this.domElement.addEventListener('mousemove', this.onMouseMove);
            this.domElement.addEventListener('mouseup', this.onMouseUp);
        }
        
        onMouseMove(event) {
            if (!this.enabled) return;
            event.preventDefault();
            
            const clientX = event.touches ? event.touches[0].clientX : event.clientX;
            const clientY = event.touches ? event.touches[0].clientY : event.clientY;
            
            if (this._state === 1) {
                this.rotateEnd.set(clientX, clientY);
                this.rotateDelta.subVectors(this.rotateEnd, this.rotateStart).multiplyScalar(this.rotateSpeed);
                this.sphericalDelta.theta -= 2 * Math.PI * this.rotateDelta.x / this.domElement.clientHeight * 0.5;
                this.sphericalDelta.phi -= 2 * Math.PI * this.rotateDelta.y / this.domElement.clientHeight * 0.5;
                this.rotateStart.copy(this.rotateEnd);
            } else if (this._state === 2) {
                this.panEnd.set(clientX, clientY);
                this.panDelta.subVectors(this.panEnd, this.panStart).multiplyScalar(this.panSpeed);
                this.pan(this.panDelta.x, this.panDelta.y);
                this.panStart.copy(this.panEnd);
            }
            
            this.update();
        }
        
        onMouseUp(event) {
            if (!this.enabled) return;
            this.domElement.removeEventListener('mousemove', this.onMouseMove);
            this.domElement.removeEventListener('mouseup', this.onMouseUp);
            this._state = 0;
        }
        
        onMouseWheel(event) {
            if (!this.enabled) return;
            event.preventDefault();
            
            const deltaY = event.deltaY;
            if (deltaY < 0) {
                this.dollyIn(this.getZoomScale());
            } else {
                this.dollyOut(this.getZoomScale());
            }
            this.update();
        }
        
        onContextMenu(event) {
            if (!this.enabled) return;
            event.preventDefault();
        }
        
        onPointerDown(event) {
            this.onMouseDown(event);
        }
        
        onPointerMove(event) {
            this.onMouseMove(event);
        }
        
        onPointerUp(event) {
            this.onMouseUp(event);
        }
        
        panLeft(distance, objectMatrix) {
            const v = new exports.Vector3();
            v.setFromMatrixColumn(objectMatrix, 0);
            v.multiplyScalar(-distance);
            this.panOffset.add(v);
        }
        
        panUp(distance, objectMatrix) {
            const v = new exports.Vector3();
            v.setFromMatrixColumn(objectMatrix, 1);
            v.multiplyScalar(distance);
            this.panOffset.add(v);
        }
        
        pan(deltaX, deltaY) {
            const offset = new exports.Vector3();
            const quat = new exports.Quaternion().setFromUnitVectors(this.object.up, new exports.Vector3(0, 1, 0));
            const quatInverse = quat.clone().invert();
            offset.copy(this.object.position).sub(this.target);
            offset.applyQuaternion(quat);
            const spherical = new exports.Spherical();
            spherical.setFromVector3(offset);
            
            const width = this.domElement.clientWidth;
            const height = this.domElement.clientHeight;
            const aspect = width / height;
            const left = this._panLeft;
            const up = this._panUp;
            const forward = new exports.Vector3();
            
            up.set(0, 1, 0);
            up.applyQuaternion(quat);
            up.normalize();
            
            forward.copy(offset).normalize();
            forward.applyQuaternion(quat);
            
            up.multiplyScalar(deltaY * spherical.radius * this.panSpeed);
            left.crossVectors(up, forward);
            left.normalize();
            left.multiplyScalar(deltaX * spherical.radius * this.panSpeed);
            this.panOffset.add(up);
            this.panOffset.add(left);
        }
        
        dollyIn(dollyScale) {
            this.scale /= dollyScale;
        }
        
        dollyOut(dollyScale) {
            this.scale *= dollyScale;
        }
        
        getZoomScale() {
            return Math.pow(0.95, this.zoomSpeed);
        }
        
        dispose() {
            this.domElement.removeEventListener('contextmenu', this.onContextMenu);
            this.domElement.removeEventListener('mousedown', this.onMouseDown);
            this.domElement.removeEventListener('wheel', this.onMouseWheel);
        }
    }
    
    // 添加事件监听器支持
    const proto = OrbitControls.prototype;
    proto.addEventListener = function(type, listener) {
        if (!this._listeners) this._listeners = {};
        if (!this._listeners[type]) this._listeners[type] = [];
        this._listeners[type].push(listener);
    };
    proto.removeEventListener = function(type, listener) {
        if (!this._listeners || !this._listeners[type]) return;
        const idx = this._listeners[type].indexOf(listener);
        if (idx >= 0) this._listeners[type].splice(idx, 1);
    };
    proto.dispatchEvent = function(event) {
        if (!this._listeners || !this._listeners[event.type]) return;
        this._listeners[event.type].forEach(listener => listener(event));
    };
    
    exports.OrbitControls = OrbitControls;
    
    })));
    
    // 挂载到全局THREE对象
    if (typeof window !== 'undefined' && window.THREE) {
        window.THREE.OrbitControls = exports.OrbitControls;
    }

