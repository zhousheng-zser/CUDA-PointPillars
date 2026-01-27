(function() {

	// �?? TrackballControls 不同，它保留了“向上”方向对�??.up（默认情况下�?? +Y）�?
	//
	// 轨道 - 鼠标左键/触摸：单指移�??
	// 缩放 - 鼠标中键或鼠标滚�?? / 触摸：双指张开或挤�??
	// 平移 - 右键，或左键 + Ctrl/Meta/Shift 键，或方向键 / 触摸：双指移�??

	const _changeEvent = {
		type: 'change'
	};
	const _startEvent = {
		type: 'start'
	};
	const _endEvent = {
		type: 'end'
	};

	class OrbitControls extends THREE.EventDispatcher {

		constructor( object, domElement ) {

			super();
			if ( domElement === undefined ) console.warn( 'THREE.OrbitControls: 第二个参数“domElement”现在是必需的�?' );
			if ( domElement === document ) console.error( 'THREE.OrbitControls: "document" 不应用作目标 "domElement"。请改用 "renderer.domElement"�??' );
			this.object = object;
			this.domElement = domElement; // 设置�?? false 可禁用此控件

			this.enabled = true; // "target" 设置焦点位置，即物体围绕其旋转的位置�??

			this.target = new THREE.Vector3(); // 可推拉的距离（仅限透视相机�??

			this.minDistance = 0;
			this.maxDistance = Infinity; // 最大缩放距离（仅限正交相机�??

			this.minZoom = 0;
			this.maxZoom = Infinity; // 垂直方向上可以旋转的最大距离，上限和下限�?
			// 范围�?? 0 �?? Math.PI 弧度�??

			this.minPolarAngle = 0; // 弧度

			this.maxPolarAngle = Math.PI; // 弧度
			// 水平轨道飞行的上限和下限�??
			// 如果设置，区�?? [ min, max ] 必须是区�?? [ - 2 PI, 2 PI ] 的子区间，且 ( max - min < 2 PI )�??

			this.minAzimuthAngle = - Infinity; // 弧度

			this.maxAzimuthAngle = Infinity; // 弧度
			// 设置�?? true 以启用阻尼（惯性）
			// 如果启用了阻尼，则必须在动画循环中调�?? controls.update()

			this.enableDamping = false;
			this.dampingFactor = 0.05; // 此选项实际上启用推拉镜头；保留为“zoom”是为了向后兼容�??
			// 设置�?? false 可禁用缩放功能�?

			this.enableZoom = true;
			this.zoomSpeed = 1.0; // 设置�?? false 可禁用旋�??

			this.enableRotate = true;
			this.rotateSpeed = 1.0; // 设置�?? false 可禁用平�??

			this.enablePan = true;
			this.panSpeed = 1.0;
			this.screenSpacePanning = true; // 如果�?? false，则垂直于世界空间方向平�?? camera.up

			this.keyPanSpeed = 7.0; // 每次按下方向键移动的像素�??
			// 设置�?? true 可自动围绕目标旋�??
			// 如果启用了自动旋转，则必须在动画循环中调�?? controls.update()

			this.autoRotate = false;
			this.autoRotateSpeed = 2.0; // 当帧率为 60 时，每圈旋转 30 �??
			四个方向�??

			this.keys = {
				左：'向左箭头'�??
				向上�??'向上箭头'
				右箭头：'ArrowRight'
				底部�??'向下箭头'
			}; // 鼠标按钮

			this.mouseButtons = {
				左：�??.鼠标.旋转�??
				中间：三只小老鼠娃娃
				右：�??.鼠标.平移
			}; // 触摸手指

			this.touches = {
				一：三.触摸.旋转�??
				二：�??.触摸.推拉�??
			}; // 用于重置

			this.target0 = this.target.clone();
			this.position0 = this.object.position.clone();
			this.zoom0 = this.object.zoom; // 关键事件的目�?? DOM 元素

			this._domElementKeyEvents = null; //
			// 公共方法
			//

			this.getPolarAngle = function () {

				返回 spherical.phi�??

			};

			this.getAzimuthalAngle = function () {

				返回 spherical.theta�??

			};

			this.listenToKeyEvents = function ( domElement ) {

				domElement.addEventListener('keydown', onKeyDown);
				this._domElementKeyEvents = domElement;

			};

			this.saveState = function () {

				scope.target0.copy( scope.target );
				scope.position0.copy( scope.object.position );
				scope.zoom0 = scope.object.zoom;

			};

			this.reset = function () {

				scope.target.copy( scope.target0 );
				scope.object.position.copy( scope.position0 );
				scope.object.zoom = scope.zoom0;
				scope.object.updateProjectionMatrix();
				scope.dispatchEvent( _changeEvent );
				作用域更�??();
				状�? = STATE.NONE;

			}; // 这个方法是公开的，但如果我们能把它设为私有方法，或许会更好…�?


			this.update = function () {

				const offset = new THREE.Vector3(); // 因此 camera.up 是轨道轴

				const quat = new THREE.Quaternion().setFromUnitVectors( object.up, new THREE.Vector3( 0, 1, 0 ) );
				const quatInverse = quat.clone().invert();
				const lastPosition = new THREE.Vector3();
				const lastQuaternion = new THREE.Quaternion();
				const twoPI = 2 * Math.PI;
				返回函数 update() {

					const position = scope.object.position;
					offset.copy(position).sub(scope.target); // 将偏移量旋转到“y轴向上”的空间

					offset.applyQuaternion(quat); // �?? z 轴绕 y 轴的角度

					spherical.setFromVector3( offset );

					如果 ( scope.autoRotate && state === STATE.NONE ) {

						rotateLeft(getAutoRotationAngle());

					}

					如果 ( scope.enableDamping ) {

						球面.θ += 球面Δ.θ * 阻尼因子;
						球面.phi += 球面Delta.phi * scope.dampingFactor;

					} 别的 {

						球面θ += 球面Δθ;
						球面.phi += 球面Delta.phi;

					} // �?? theta 限制在所需范围�??


					�?? min =scope.minAzimuthAngle;
					let max = scope.maxAzimuthAngle;

					如果 ( isFinite( min ) && isFinite( max ) ) {

						如果 ( min < - Math.PI ) min += twoPI; 否则如果 ( min > Math.PI ) min -= twoPI;
						如果 ( max < - Math.PI ) max += twoPI; 否则如果 ( max > Math.PI ) max -= twoPI;

						如果（最小�? <= 最大值）{

							spherical.theta = Math.max( min, Math.min( max, spherical.theta ) );

						} 别的 {

							spherical.theta = spherical.theta > ( min + max ) / 2 ? Math.max( min, spherical.theta ) : Math.min( max, spherical.theta );

						}

					} // �?? phi 限制在所需范围�??


					spherical.phi = Math.max( scope.minPolarAngle, Math.min( scope.maxPolarAngle, spherical.phi ) );
					球形.makeSafe();
					spherical.radius *= scale; // 将半径限制在所需范围�??

					spherical.radius = Math.max( scope.minDistance, Math.min( scope.maxDistance, spherical.radius ) ); // 将目标移动到平移位置

					如果 ( scope.enableDamping === true ) {

						scope.target.addScaledVector( panOffset, scope.dampingFactor );

					} 别的 {

						scope.target.add( panOffset );

					}

					offset.setFromSpherical( spherical ); // 将偏移量旋转回“相机向上向量向上”的空间

					offset.applyQuaternion(quatInverse);
					position.copy(scope.target).add(offset);
					scope.object.lookAt(scope.target);

					如果 ( scope.enableDamping === true ) {

						sphericalDelta.theta *= 1 - scope.dampingFactor;
						sphericalDelta.phi *= 1 - scope.dampingFactor;
						panOffset.multiplyScalar(1 - scope.dampingFactor);

					} 别的 {

						sphericalDelta.set( 0, 0, 0 );
						panOffset.set(0, 0, 0);

					}

					scale = 1; // 更新条件为：
					// min(相机位移，相机旋转角度（弧度�??)^2 > EPS
					// 使用小角度近�?? cos(x/2) = 1 - x^2 / 8

					如果 ( zoomChanged || lastPosition.distanceToSquared( scope.object.position ) > EPS || 8 * ( 1 - lastQuaternion.dot( scope.object.quaternion ) ) > EPS ) {

						scope.dispatchEvent( _changeEvent );
						lastPosition.copy( scope.object.position );
						lastQuaternion.copy( scope.object.quaternion );
						zoomChanged = false;
						返回 true�??

					}

					返回 false�??

				};

			}();

			this.dispose = function () {

				scope.domElement.removeEventListener( 'contextmenu', onContextMenu );
				scope.domElement.removeEventListener('pointerdown', onPointerDown);
				scope.domElement.removeEventListener('wheel', onMouseWheel);
				scope.domElement.removeEventListener('touchstart', onTouchStart);
				scope.domElement.removeEventListener('touchend', onTouchEnd);
				scope.domElement.removeEventListener('touchmove', onTouchMove);
				scope.domElement.ownerDocument.removeEventListener('pointermove', onPointerMove);
				scope.domElement.ownerDocument.removeEventListener('pointerup', onPointerUp);

				if ( scope._domElementKeyEvents !== null ) {

					scope._domElementKeyEvents.removeEventListener('keydown', onKeyDown);

				} //scope.dispatchEvent( { type: 'dispose' } ); // 这部分应该添加到这里吗？

			}; //
			// 内部结构
			//


			const scope = this;
			const STATE = {
				无：-1�??
				旋转�??0�??
				多莉�??1�??
				PAN�??2�??
				TOUCH_ROTATE: 3,
				TOUCH_PAN�??4�??
				TOUCH_DOLLY_PAN: 5,
				TOUCH_DOLLY_ROTATE: 6
			};
			let state = STATE.NONE;
			const EPS = 0.000001; // 当前位置（球坐标系）

			const spherical = new THREE.Spherical();
			const sphericalDelta = new THREE.Spherical();
			�?? scale = 1�??
			const panOffset = new THREE.Vector3();
			let zoomChanged = false;
			const rotateStart = new THREE.Vector2();
			const rotateEnd = new THREE.Vector2();
			const rotateDelta = new THREE.Vector2();
			const panStart = new THREE.Vector2();
			const panEnd = new THREE.Vector2();
			const panDelta = new THREE.Vector2();
			const dollyStart = new THREE.Vector2();
			const dollyEnd = new THREE.Vector2();
			const dollyDelta = new THREE.Vector2();

			function getAutoRotationAngle() {

				返回 2 * Math.PI / 60 / 60 * scope.autoRotateSpeed;

			}

			function getZoomScale() {

				返回 Math.pow(0.95, scope.zoomSpeed);

			}

			function rotateLeft( angle ) {

				sphericalDelta.theta -= 角度;

			}

			function rotateUp( angle ) {

				球面Δ.phi -= 角度�??

			}

			const panLeft = function () {

				const v = new THREE.Vector3();
				返回函数 panLeft( distance, objectMatrix ) {

					v.setFromMatrixColumn( objectMatrix, 0 ); // 获取 objectMatrix �?? X �??

					v.multiplyScalar( - distance );
					panOffset.add( v );

				};

			}();

			const panUp = function () {

				const v = new THREE.Vector3();
				返回函数 panUp(距离, objectMatrix) {

					如果 ( scope.screenSpacePanning === true ) {

						v.setFromMatrixColumn( objectMatrix, 1 );

					} 别的 {

						v.setFromMatrixColumn( objectMatrix, 0 );
						v.crossVectors(scope.object.up, v);

					}

					v.multiplyScalar(距离);
					panOffset.add( v );

				};

			}(); // deltaX �?? deltaY 的单位是像素；向右和向下为正值�?


			const pan = function () {

				const offset = new THREE.Vector3();
				返回函数 pan( deltaX, deltaY ) {

					const element = scope.domElement;

					如果 ( scope.object.isPerspectiveCamera ) {

						// 看法
						const position = scope.object.position;
						offset.copy(position).sub(scope.target);
						let targetDistance = offset.length(); // 视野的一半是从屏幕中心到顶部的距�??

						targetDistance *= Math.tan( scope.object.fov / 2 * Math.PI / 180.0 ); // 这里只使�?? clientHeight，所以宽高比不会影响速度

						panLeft( 2 * deltaX * targetDistance / element.clientHeight, scope.object.matrix );
						panUp( 2 * deltaY * targetDistance / element.clientHeight, scope.object.matrix );

					} else if ( scope.object.isOrthographicCamera ) {

						正交
						panLeft( deltaX * ( scope.object.right - scope.object.left ) / scope.object.zoom / element.clientWidth, scope.object.matrix );
						panUp(deltaY * (scope.object.top - scope.object.bottom) / scope.object.zoom / element.clientHeight, scope.object.matrix);

					} 别的 {

						// 相机既非正交相机也非透视相机
						console.warn('警告：OrbitControls.js 遇到未知的相机类�?? - 平移已禁用�?');
						scope.enablePan = false;

					}

				};

			}();

			function dollyOut( dollyScale ) {

				如果 ( scope.object.isPerspectiveCamera ) {

					比例�?? /= 多莉比例尺；

				} else if ( scope.object.isOrthographicCamera ) {

					scope.object.zoom = Math.max( scope.minZoom, Math.min( scope.maxZoom, scope.object.zoom * dollyScale ) );
					scope.object.updateProjectionMatrix();
					zoomChanged = true;

				} 别的 {

					console.warn('警告：OrbitControls.js 遇到未知的摄像机类型 - 推拉/变焦功能已禁用�?');
					scope.enableZoom = false;

				}

			}

			function dollyIn( dollyScale ) {

				如果 ( scope.object.isPerspectiveCamera ) {

					比例�?? *= 多莉比例尺；

				} else if ( scope.object.isOrthographicCamera ) {

					scope.object.zoom = Math.max( scope.minZoom, Math.min( scope.maxZoom, scope.object.zoom / dollyScale ) );
					scope.object.updateProjectionMatrix();
					zoomChanged = true;

				} 别的 {

					console.warn('警告：OrbitControls.js 遇到未知的摄像机类型 - 推拉/变焦功能已禁用�?');
					scope.enableZoom = false;

				}

			} //
			// 事件回调 - 更新对象状�?
			//


			function handleMouseDownRotate( event ) {

				rotateStart.set(event.clientX, event.clientY);

			}

			function handleMouseDownDolly( event ) {

				dollyStart.set(event.clientX, event.clientY);

			}

			function handleMouseDownPan( event ) {

				panStart.set(event.clientX, event.clientY);

			}

			function handleMouseMoveRotate( event ) {

				rotateEnd.set( event.clientX, event.clientY );
				rotateDelta.subVectors( rotateEnd, rotateStart ).multiplyScalar( scope.rotateSpeed );
				const element = scope.domElement;
				rotateLeft( 2 * Math.PI * rotateDelta.x / element.clientHeight ); // 是的，高�??

				rotateUp( 2 * Math.PI * rotateDelta.y / element.clientHeight );
				rotateStart.copy( rotateEnd );
				作用域更�??();

			}

			function handleMouseMoveDolly( event ) {

				dollyEnd.set(event.clientX, event.clientY);
				dollyDelta.subVectors( dollyEnd, dollyStart );

				如果 ( dollyDelta.y > 0 ) {

					dollyOut(getZoomScale());

				} else if ( dollyDelta.y < 0 ) {

					dollyIn(getZoomScale());

				}

				dollyStart.copy( dollyEnd );
				作用域更�??();

			}

			function handleMouseMovePan( event ) {

				panEnd.set(event.clientX, event.clientY);
				panDelta.subVectors( panEnd, panStart ).multiplyScalar( scope.panSpeed );
				平移（panDelta.x，panDelta.y）；
				panStart.copy( panEnd );
				作用域更�??();

			}

			function handleMouseUp() { // 空操�??
			}

			function handleMouseWheel( event ) {

				如果 ( event.deltaY < 0 ) {

					dollyIn(getZoomScale());

				} else if ( event.deltaY > 0 ) {

					dollyOut(getZoomScale());

				}

				作用域更�??();

			}

			function handleKeyDown( event ) {

				let needsUpdate = false;

				switch ( event.code ) {

					case scope.keys.UP:
						pan(0, scope.keyPanSpeed);
						needsUpdate = true;
						休息;

					案例范围.�??.底部�??
						pan( 0, - scope.keyPanSpeed );
						needsUpdate = true;
						休息;

					case scope.keys.LEFT:
						pan( scope.keyPanSpeed, 0 );
						needsUpdate = true;
						休息;

					case scope.keys.RIGHT:
						pan( - scope.keyPanSpeed, 0 );
						needsUpdate = true;
						休息;

				}

				如果（需要更新）{

					// 防止浏览器在光标键上滚动
					event.preventDefault();
					作用域更�??();

				}

			}

			function handleTouchStartRotate( event ) {

				如果 ( event.touches.length == 1 ) {

					rotateStart.set(event.touches[0].pageX, event.touches[0].pageY);

				} 别的 {

					const x = 0.5 * ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX );
					const y = 0.5 * ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY );
					rotateStart.set( x, y );

				}

			}

			function handleTouchStartPan( event ) {

				如果 ( event.touches.length == 1 ) {

					panStart.set(event.touches[0].pageX, event.touches[0].pageY);

				} 别的 {

					const x = 0.5 * ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX );
					const y = 0.5 * ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY );
					panStart.set( x, y );

				}

			}

			function handleTouchStartDolly( event ) {

				const dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
				const dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
				const distance = Math.sqrt( dx * dx + dy * dy );
				dollyStart.set(0, distance);

			}

			function handleTouchStartDollyPan( event ) {

				if ( scope.enableZoom ) handleTouchStartDolly( event );
				if ( scope.enablePan ) handleTouchStartPan( event );

			}

			function handleTouchStartDollyRotate( event ) {

				if ( scope.enableZoom ) handleTouchStartDolly( event );
				如果 ( scope.enableRotate ) handleTouchStartRotate( event );

			}

			function handleTouchMoveRotate( event ) {

				如果 ( event.touches.length == 1 ) {

					rotateEnd.set(event.touches[0].pageX, event.touches[0].pageY);

				} 别的 {

					const x = 0.5 * ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX );
					const y = 0.5 * ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY );
					rotateEnd.set( x, y );

				}

				rotateDelta.subVectors( rotateEnd, rotateStart ).multiplyScalar( scope.rotateSpeed );
				const element = scope.domElement;
				rotateLeft( 2 * Math.PI * rotateDelta.x / element.clientHeight ); // 是的，高�??

				rotateUp( 2 * Math.PI * rotateDelta.y / element.clientHeight );
				rotateStart.copy( rotateEnd );

			}

			function handleTouchMovePan( event ) {

				如果 ( event.touches.length == 1 ) {

					panEnd.set(event.touches[0].pageX, event.touches[0].pageY);

				} 别的 {

					const x = 0.5 * ( event.touches[ 0 ].pageX + event.touches[ 1 ].pageX );
					const y = 0.5 * ( event.touches[ 0 ].pageY + event.touches[ 1 ].pageY );
					panEnd.set( x, y );

				}

				panDelta.subVectors( panEnd, panStart ).multiplyScalar( scope.panSpeed );
				平移（panDelta.x，panDelta.y）；
				panStart.copy( panEnd );

			}

			function handleTouchMoveDolly( event ) {

				const dx = event.touches[ 0 ].pageX - event.touches[ 1 ].pageX;
				const dy = event.touches[ 0 ].pageY - event.touches[ 1 ].pageY;
				const distance = Math.sqrt( dx * dx + dy * dy );
				dollyEnd.set(0, distance);
				dollyDelta.set( 0, Math.pow( dollyEnd.y / dollyStart.y, scope.zoomSpeed ) );
				dollyOut(dollyDelta.y);
				dollyStart.copy( dollyEnd );

			}

			function handleTouchMoveDollyPan( event ) {

				if ( scope.enableZoom ) handleTouchMoveDolly( event );
				if ( scope.enablePan ) handleTouchMovePan( event );

			}

			function handleTouchMoveDollyRotate( event ) {

				if ( scope.enableZoom ) handleTouchMoveDolly( event );
				如果 ( scope.enableRotate ) handleTouchMoveRotate( event );

			}

			function handleTouchEnd() { // 空操�??
			} //
			// 事件处理程序 - FSM：监听事件并重置状�?
			//


			function onPointerDown( event ) {

				如果 ( scope.enabled === false ) 返回;

				switch ( event.pointerType ) {

					案例“mouse”：
					案例“pen”：
						onMouseDown(事件);
						休息;
        // TODO 触摸

				}

			}

			function onPointerMove( event ) {

				如果 ( scope.enabled === false ) 返回;

				switch ( event.pointerType ) {

					案例“mouse”：
					案例“pen”：
						onMouseMove(事件);
						休息;
        // TODO 触摸

				}

			}

			function onPointerUp( event ) {

				switch ( event.pointerType ) {

					案例“mouse”：
					案例“pen”：
						onMouseUp(事件);
						休息;
        // TODO 触摸

				}

			}

			function onMouseDown( event ) {

				// 阻止浏览器滚动�?
				event.preventDefault(); // 由于上面已经调用�?? preventDefault，所以需要手动设置焦点�?
				// 阻止浏览器自动设置�?

				scope.domElement.focus ? scope.domElement.focus() : window.focus();
				let mouseAction;

				switch ( event.button ) {

					案例 0�??
						mouseAction = scope.mouseButtons.LEFT;
						休息;

					案例1�??
						mouseAction = scope.mouseButtons.MIDDLE;
						休息;

					案例二：
						mouseAction = scope.mouseButtons.RIGHT;
						休息;

					默认�??
						mouseAction = -1;

				}

				switch ( mouseAction ) {

					案例 THREE.MOUSE.DOLLY�??
						如果 ( scope.enableZoom === false ) 返回;
						handleMouseDownDolly(事件);
						状�? = STATE.DOLLY;
						休息;

					案例 THREE.MOUSE.ROTATE�??
						如果 ( event.ctrlKey || event.metaKey || event.shiftKey ) {

							如果 ( scope.enablePan === false ) 返回;
							handleMouseDownPan(事件);
							状�? = STATE.PAN;

						} 别的 {

							如果 ( scope.enableRotate === false ) 返回;
							handleMouseDownRotate(事件);
							状�? = STATE.ROTATE;

						}

						休息;

					案例 THREE.MOUSE.PAN�??
						如果 ( event.ctrlKey || event.metaKey || event.shiftKey ) {

							如果 ( scope.enableRotate === false ) 返回;
							handleMouseDownRotate(事件);
							状�? = STATE.ROTATE;

						} 别的 {

							如果 ( scope.enablePan === false ) 返回;
							handleMouseDownPan(事件);
							状�? = STATE.PAN;

						}

						休息;

					默认�??
						状�? = STATE.NONE;

				}

				如果 (状�? !== STATE.NONE) {

					scope.domElement.ownerDocument.addEventListener('pointermove', onPointerMove);
					scope.domElement.ownerDocument.addEventListener('pointerup', onPointerUp);
					scope.dispatchEvent( _startEvent );

				}

			}

			function onMouseMove( event ) {

				如果 ( scope.enabled === false ) 返回;
				event.preventDefault();

				switch ( 状�? ) {

					case STATE.ROTATE:
						如果 ( scope.enableRotate === false ) 返回;
						handleMoveRotate(事件);
						休息;

					案例 STATE.DOLLY�??
						如果 ( scope.enableZoom === false ) 返回;
						handleMouseMoveDolly(事件);
						休息;

					案例 STATE.PAN�??
						如果 ( scope.enablePan === false ) 返回;
						handleMovePan(事件);
						休息;

				}

			}

			function onMouseUp( event ) {

				scope.domElement.ownerDocument.removeEventListener('pointermove', onPointerMove);
				scope.domElement.ownerDocument.removeEventListener('pointerup', onPointerUp);
				如果 ( scope.enabled === false ) 返回;
				handleMouseUp(事件);
				scope.dispatchEvent( _endEvent );
				状�? = STATE.NONE;

			}

			function onMouseWheel( event ) {

				如果 ( scope.enabled === false || scope.enableZoom === false || state !== STATE.NONE && state !== STATE.ROTATE ) 返回;
				event.preventDefault();
				scope.dispatchEvent( _startEvent );
				处理鼠标滚轮（事件）�??
				scope.dispatchEvent( _endEvent );

			}

			function onKeyDown( event ) {

				如果 ( scope.enabled === false || scope.enablePan === false ) 返回;
				handleKeyDown(事件);

			}

			function onTouchStart( event ) {

				如果 ( scope.enabled === false ) 返回;
				event.preventDefault(); // 防止滚动

				switch ( event.touches.length ) {

					案例1�??
						switch ( scope.touches.ONE ) {

							案例 THREE.TOUCH.ROTATE�??
								如果 ( scope.enableRotate === false ) 返回;
								handleTouchStartRoot(事件);
								状�? = STATE.TOUCH_ROTATE;
								休息;

							案例 THREE.TOUCH.PAN�??
								如果 ( scope.enablePan === false ) 返回;
								handleTouchStartPan(事件);
								状�? = STATE.TOUCH_PAN;
								休息;

							默认�??
								状�? = STATE.NONE;

						}

						休息;

					案例二：
						switch ( scope.touches.TWO ) {

							案例 THREE.TOUCH.DOLLY_PAN�??
								如果 ( scope.enableZoom === false && scope.enablePan === false ) 返回;
								handleTouchStartDollyPan(事件);
								状�? = STATE.TOUCH_DOLLY_PAN;
								休息;

							案例 THREE.TOUCH.DOLLY_ROTATE�??
								如果 ( scope.enableZoom === false && scope.enableRotate === false ) 返回;
								handleTouchStartDollyRotate(事件);
								状�? = STATE.TOUCH_DOLLY_ROTATE;
								休息;

							默认�??
								状�? = STATE.NONE;

						}

						休息;

					默认�??
						状�? = STATE.NONE;

				}

				如果 (状�? !== STATE.NONE) {

					scope.dispatchEvent( _startEvent );

				}

			}

			function onTouchMove( event ) {

				如果 ( scope.enabled === false ) 返回;
				event.preventDefault(); // 防止滚动

				switch ( 状�? ) {

					case STATE.TOUCH_ROTATE:
						如果 ( scope.enableRotate === false ) 返回;
						handleTouchMoveRotate(事件);
						作用域更�??();
						休息;

					case STATE.TOUCH_PAN:
						如果 ( scope.enablePan === false ) 返回;
						handleTouchMovePan(事件);
						作用域更�??();
						休息;

					case STATE.TOUCH_DOLLY_PAN:
						如果 ( scope.enableZoom === false && scope.enablePan === false ) 返回;
						handleTouchMoveDollyPan(事件);
						作用域更�??();
						休息;

					case STATE.TOUCH_DOLLY_ROTATE:
						如果 ( scope.enableZoom === false && scope.enableRotate === false ) 返回;
						handleTouchMoveDollyRotate(事件);
						作用域更�??();
						休息;

					默认�??
						状�? = STATE.NONE;

				}

			}

			function onTouchEnd( event ) {

				如果 ( scope.enabled === false ) 返回;
				handleTouchEnd(事件);
				scope.dispatchEvent( _endEvent );
				状�? = STATE.NONE;

			}

			函数 onContextMenu( 事件 ) {

				如果 ( scope.enabled === false ) 返回;
				event.preventDefault();

			} //


			scope.domElement.addEventListener( 'contextmenu', onContextMenu );
			scope.domElement.addEventListener('pointerdown', onPointerDown);
			scope.domElement.addEventListener('wheel', onMouseWheel, {
				被动：否
			} );
			scope.domElement.addEventListener('touchstart', onTouchStart, {
				被动：否
			} );
			scope.domElement.addEventListener('touchend', onTouchEnd);
			scope.domElement.addEventListener('touchmove', onTouchMove, {
				被动：否
			} ); // 强制在开始时更新

			this.update();

		}

	} // 这组控件可执行旋转、推拉（缩放）和平移�??
	// �?? TrackballControls 不同，它保留了“向上”方向对�??.up（默认情况下�?? +Y）�?
	// 这与 OrbitControls 非常相似，OrbitControls 是另一组触摸行为�?
	//
	// 旋转 - 右键，或左键 + Ctrl/Meta/Shift �?? / 触摸：双指旋�??
	// 缩放 - 鼠标中键或鼠标滚�?? / 触摸：双指张开或挤�??
	// 平移 - 鼠标左键或方向键 / 触摸：单指移�??


	class MapControls extends OrbitControls {

		constructor( object, domElement ) {

			super(object, domElement);
			this.screenSpacePanning = false; // 垂直于世界空间方向平�?? camera.up

			this.mouseButtons.LEFT = THREE.MOUSE.PAN;
			this.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
			this.touches.ONE = THREE.TOUCH.PAN;
			this.touches.TWO = THREE.TOUCH.DOLLY_ROTATE;

		}

	}

	THREE.MapControls = MapControls;
	THREE.OrbitControls = OrbitControls;

} )();