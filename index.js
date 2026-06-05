var Vector3 = /** @class */ (function () {
    function Vector3(x, y, z) {
        if (x === void 0) { x = 0; }
        if (y === void 0) { y = 0; }
        if (z === void 0) { z = 0; }
        this.x = x;
        this.y = y;
        this.z = z;
    }
    Vector3.prototype.add = function (v) {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    };
    Vector3.prototype.mult = function (s) {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    };
    Vector3.prototype.mag = function () {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    };
    Vector3.prototype.magSq = function () {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    };
    Vector3.prototype.copy = function () {
        return new Vector3(this.x, this.y, this.z);
    };
    return Vector3;
}());
var Body = /** @class */ (function () {
    function Body(pos, vel, mass, isCenter) {
        if (isCenter === void 0) { isCenter = false; }
        this.pos = pos;
        this.vel = vel;
        this.acc = new Vector3(0, 0, 0);
        this.mass = mass;
        this.isCenter = isCenter;
    }
    return Body;
}());
var OctreeNode = /** @class */ (function () {
    function OctreeNode(center, size) {
        this.center = center;
        this.size = size;
        this.children = null;
        this.body = null;
        this.mass = 0;
        this.com = new Vector3(0, 0, 0);
        this.isLeaf = true;
    }
    OctreeNode.prototype.getOctant = function (pos) {
        var index = 0;
        if (pos.x >= this.center.x)
            index += 4;
        if (pos.y >= this.center.y)
            index += 2;
        if (pos.z >= this.center.z)
            index += 1;
        return index;
    };
    OctreeNode.prototype.insert = function (body) {
        if (this.mass === 0) {
            this.body = body;
            this.mass = body.mass;
            this.com = body.pos.copy();
            return;
        }
        if (this.isLeaf && this.body !== null) {
            this.subdivide();
            var existingBody = this.body;
            this.body = null;
            this.insertIntoChildren(existingBody);
        }
        this.insertIntoChildren(body);
        this.mass += body.mass;
        this.com.x = (this.com.x * (this.mass - body.mass) + body.pos.x * body.mass) / this.mass;
        this.com.y = (this.com.y * (this.mass - body.mass) + body.pos.y * body.mass) / this.mass;
        this.com.z = (this.com.z * (this.mass - body.mass) + body.pos.z * body.mass) / this.mass;
    };
    OctreeNode.prototype.insertIntoChildren = function (body) {
        var octant = this.getOctant(body.pos);
        if (this.children) {
            this.children[octant].insert(body);
        }
    };
    OctreeNode.prototype.subdivide = function () {
        this.children = new Array(8);
        var halfSize = this.size / 2;
        var quarterSize = this.size / 4;
        var offsets = [
            [-1, -1, -1], // 0: -X, -Y, -Z
            [-1, -1, 1], // 1: -X, -Y, +Z
            [-1, 1, -1], // 2: -X, +Y, -Z
            [-1, 1, 1], // 3: -X, +Y, +Z
            [1, -1, -1], // 4: +X, -Y, -Z
            [1, -1, 1], // 5: +X, -Y, +Z
            [1, 1, -1], // 6: +X, +Y, -Z
            [1, 1, 1] // 7: +X, +Y, +Z
        ];
        for (var i = 0; i < 8; i++) {
            var dx = offsets[i][0] * quarterSize;
            var dy = offsets[i][1] * quarterSize;
            var dz = offsets[i][2] * quarterSize;
            var childCenter = new Vector3(this.center.x + dx, this.center.y + dy, this.center.z + dz);
            this.children[i] = new OctreeNode(childCenter, halfSize);
        }
        this.isLeaf = false;
    };
    OctreeNode.prototype.calculateForce = function (body, theta, G, softening) {
        if (this.mass === 0)
            return new Vector3(0, 0, 0);
        var dx = this.com.x - body.pos.x;
        var dy = this.com.y - body.pos.y;
        var dz = this.com.z - body.pos.z;
        var distSq = dx * dx + dy * dy + dz * dz + softening * softening;
        var dist = Math.sqrt(distSq);
        if (this.isLeaf || (this.size / dist < theta)) {
            var force = G * this.mass / distSq;
            return new Vector3(force * dx / dist, force * dy / dist, force * dz / dist);
        }
        else {
            var totalForce = new Vector3(0, 0, 0);
            if (this.children) {
                for (var _i = 0, _a = this.children; _i < _a.length; _i++) {
                    var child = _a[_i];
                    if (child && child.mass > 0) {
                        var f = child.calculateForce(body, theta, G, softening);
                        totalForce = totalForce.add(f);
                    }
                }
            }
            return totalForce;
        }
    };
    OctreeNode.prototype.countNodes = function () {
        var count = 1;
        if (this.children) {
            for (var _i = 0, _a = this.children; _i < _a.length; _i++) {
                var child = _a[_i];
                if (child)
                    count += child.countNodes();
            }
        }
        return count;
    };
    OctreeNode.prototype.getBoundaries = function () {
        var bounds = [{
                center: this.center,
                size: this.size
            }];
        if (this.children) {
            for (var _i = 0, _a = this.children; _i < _a.length; _i++) {
                var child = _a[_i];
                if (child && child.mass > 0) {
                    bounds.push.apply(bounds, child.getBoundaries());
                }
            }
        }
        return bounds;
    };
    return OctreeNode;
}());
var GalaxySimulation = /** @class */ (function () {
    function GalaxySimulation() {
        this.bodies = [];
        this.N = 2000;
        this.theta = 0.5;
        this.dt = 0.005;
        this.G = 0.1;
        this.softening = 0.01;
        this.paused = false;
        this.showTree = false;
        this.showTrails = false;
        this.root = null;
        this.armCount = 3;
        this.hasCenterMass = true;
        this.centerMass = 10000;
        this.positions = new Float32Array(this.N * 3);
        this.colors = new Float32Array(this.N * 3);
        this.sizes = new Float32Array(this.N);
        this.trailPositions = new Float32Array(this.N * 3 * 20);
        this.trailHistory = [];
        this.initThree();
        this.initGalaxy();
        this.setupUI();
        this.animate();
    }
    GalaxySimulation.prototype.initThree = function () {
        var _this = this;
        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x000000, 0.02);
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.camera.position.set(0, 20, 40);
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        var container = document.getElementById('canvas-container');
        if (container) {
            container.appendChild(this.renderer.domElement);
        }
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.autoRotate = true;
        this.controls.autoRotateSpeed = 0.5;
        this.geometry = new THREE.BufferGeometry();
        this.geometry.setAttribute('position', new THREE.BufferAttribute(this.positions, 3));
        this.geometry.setAttribute('color', new THREE.BufferAttribute(this.colors, 3));
        this.geometry.setAttribute('size', new THREE.BufferAttribute(this.sizes, 1));
        this.material = new THREE.PointsMaterial({
            size: 0.2,
            vertexColors: true,
            blending: THREE.AdditiveBlending,
            depthWrite: false,
            transparent: true
        });
        this.points = new THREE.Points(this.geometry, this.material);
        this.scene.add(this.points);
        this.treeLines = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x4f46e5, transparent: true, opacity: 0.3 }));
        this.scene.add(this.treeLines);
        this.trailGeometry = new THREE.BufferGeometry();
        this.trailGeometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
        this.trails = new THREE.LineSegments(this.trailGeometry, new THREE.LineBasicMaterial({
            color: 0x6366f1,
            transparent: true,
            opacity: 0.2,
            blending: THREE.AdditiveBlending
        }));
        this.scene.add(this.trails);
        this.trails.visible = false;
        window.addEventListener('resize', function () { return _this.onWindowResize(); });
    };
    GalaxySimulation.prototype.initGalaxy = function () {
        this.bodies = [];
        var armSpread = 0.3;
        if (this.hasCenterMass) {
            var centerBody = new Body(new Vector3(0, 0, 0), // Позиция в центре
            new Vector3(0, 0, 0), // Нулевая скорость (неподвижна)
            this.centerMass, true // Флаг isCenter
            );
            this.bodies.push(centerBody);
        }
        for (var i = 0; i < this.N; i++) {
            var armIndex = i % this.armCount;
            var armOffset = (armIndex / this.armCount) * Math.PI * 2;
            var dist = Math.random() * 15 + 2;
            var angle = armOffset + dist * 0.3 + (Math.random() - 0.5) * armSpread;
            var x = Math.cos(angle) * dist;
            var z = Math.sin(angle) * dist;
            var y = (Math.random() - 0.5) * 2 * Math.exp(-dist / 10);
            var totalMass = this.hasCenterMass ? this.centerMass : 1000;
            var v = Math.sqrt(this.G * totalMass / dist) * 0.8;
            var vx = -Math.sin(angle) * v;
            var vz = Math.cos(angle) * v;
            var pos = new Vector3(x, y, z);
            var vel = new Vector3(vx, (Math.random() - 0.5) * 0.1, vz);
            var mass = 1.0 + Math.random() * 2.0;
            this.bodies.push(new Body(pos, vel, mass));
        }
        this.positions.fill(0);
        this.colors.fill(0);
        this.sizes.fill(0);
        this.trailHistory = [];
        for (var i = 0; i < this.N; i++) {
            this.trailHistory[i] = [];
        }
        this.geometry.setDrawRange(0, this.N);
        this.updateGeometry();
    };
    GalaxySimulation.prototype.buildOctree = function () {
        var minX = Infinity, maxX = -Infinity;
        var minY = Infinity, maxY = -Infinity;
        var minZ = Infinity, maxZ = -Infinity;
        for (var _i = 0, _a = this.bodies; _i < _a.length; _i++) {
            var body = _a[_i];
            minX = Math.min(minX, body.pos.x);
            maxX = Math.max(maxX, body.pos.x);
            minY = Math.min(minY, body.pos.y);
            maxY = Math.max(maxY, body.pos.y);
            minZ = Math.min(minZ, body.pos.z);
            maxZ = Math.max(maxZ, body.pos.z);
        }
        var center = new Vector3((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
        var size = Math.max(maxX - minX, maxY - minY, maxZ - minZ) * 1.1;
        this.root = new OctreeNode(center, size);
        for (var _b = 0, _c = this.bodies; _b < _c.length; _b++) {
            var body = _c[_b];
            this.root.insert(body);
        }
        return this.root.countNodes();
    };
    GalaxySimulation.prototype.updatePhysics = function () {
        var calcStart = performance.now();
        var nodeCount = this.buildOctree();
        var nodeCountEl = document.getElementById('node-count');
        if (nodeCountEl)
            nodeCountEl.textContent = nodeCount.toString();
        for (var _i = 0, _a = this.bodies; _i < _a.length; _i++) {
            var body = _a[_i];
            if (this.root) {
                body.acc = this.root.calculateForce(body, this.theta, this.G, this.softening);
            }
        }
        var calcTime = performance.now() - calcStart;
        var calcTimeEl = document.getElementById('calc-time');
        if (calcTimeEl)
            calcTimeEl.textContent = calcTime.toFixed(1) + 'ms';
        var totalEnergy = 0;
        var totalMass = 0;
        for (var _b = 0, _c = this.bodies; _b < _c.length; _b++) {
            var body = _c[_b];
            if (body.isCenter) {
                body.vel = new Vector3(0, 0, 0);
                body.acc = new Vector3(0, 0, 0);
                totalMass += body.mass;
                continue;
            }
            body.vel = body.vel.add(body.acc.mult(this.dt));
            body.pos = body.pos.add(body.vel.mult(this.dt));
            var vSq = body.vel.magSq();
            totalEnergy += 0.5 * body.mass * vSq;
            totalMass += body.mass;
        }
        if (this.centerSphere) {
            this.centerSphere.visible = this.hasCenterMass;
        }
        var energyEl = document.getElementById('energy');
        if (energyEl)
            energyEl.textContent = totalEnergy.toFixed(2);
        this.updateGeometry();
        if (this.showTree)
            this.updateTreeVisualization();
        if (this.showTrails)
            this.updateTrails();
    };
    GalaxySimulation.prototype.updateGeometry = function () {
        var startIndex = this.hasCenterMass ? 1 : 0;
        var visualN = this.N;
        var positions = this.geometry.attributes.position.array;
        var colors = this.geometry.attributes.color.array;
        var sizes = this.geometry.attributes.size.array;
        for (var i = 0; i < visualN; i++) {
            var bodyIndex = i + startIndex;
            var body = this.bodies[bodyIndex];
            positions[i * 3] = body.pos.x;
            positions[i * 3 + 1] = body.pos.y;
            positions[i * 3 + 2] = body.pos.z;
            var speed = body.vel.mag();
            var t = Math.min(speed / 2, 1);
            colors[i * 3] = 0.3 + t * 0.7;
            colors[i * 3 + 1] = 0.5 + t * 0.3;
            colors[i * 3 + 2] = 1.0 - t * 0.5;
            sizes[i] = 0.3 + body.mass * 0.2;
        }
        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.attributes.color.needsUpdate = true;
        this.geometry.attributes.size.needsUpdate = true;
    };
    GalaxySimulation.prototype.updateTreeVisualization = function () {
        if (!this.root)
            return;
        var bounds = this.root.getBoundaries();
        var vertices = [];
        for (var _i = 0, bounds_1 = bounds; _i < bounds_1.length; _i++) {
            var b = bounds_1[_i];
            var hs = b.size / 2;
            var cx = b.center.x;
            var cy = b.center.y;
            var cz = b.center.z;
            var corners = [
                [cx - hs, cy - hs, cz - hs], [cx + hs, cy - hs, cz - hs],
                [cx + hs, cy - hs, cz - hs], [cx + hs, cy + hs, cz - hs],
                [cx + hs, cy + hs, cz - hs], [cx - hs, cy + hs, cz - hs],
                [cx - hs, cy + hs, cz - hs], [cx - hs, cy - hs, cz - hs],
                [cx - hs, cy - hs, cz + hs], [cx + hs, cy - hs, cz + hs],
                [cx + hs, cy - hs, cz + hs], [cx + hs, cy + hs, cz + hs],
                [cx + hs, cy + hs, cz + hs], [cx - hs, cy + hs, cz + hs],
                [cx - hs, cy + hs, cz + hs], [cx - hs, cy - hs, cz + hs],
                [cx - hs, cy - hs, cz - hs], [cx - hs, cy - hs, cz + hs],
                [cx + hs, cy - hs, cz - hs], [cx + hs, cy - hs, cz + hs],
                [cx + hs, cy + hs, cz - hs], [cx + hs, cy + hs, cz + hs],
                [cx - hs, cy + hs, cz - hs], [cx - hs, cy + hs, cz + hs]
            ];
            for (var _a = 0, corners_1 = corners; _a < corners_1.length; _a++) {
                var c = corners_1[_a];
                vertices.push.apply(vertices, c);
            }
        }
        this.treeLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    };
    GalaxySimulation.prototype.updateTrails = function () {
        for (var i = 0; i < this.N; i++) {
            var body = this.bodies[i];
            this.trailHistory[i].push(body.pos.copy());
            if (this.trailHistory[i].length > 20) {
                this.trailHistory[i].shift();
            }
        }
        var vertices = [];
        for (var i = 0; i < this.N; i++) {
            var history_1 = this.trailHistory[i];
            for (var j = 0; j < history_1.length - 1; j++) {
                vertices.push(history_1[j].x, history_1[j].y, history_1[j].z, history_1[j + 1].x, history_1[j + 1].y, history_1[j + 1].z);
            }
        }
        this.trails.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    };
    GalaxySimulation.prototype.setupUI = function () {
        var _this = this;
        var armsInput = document.getElementById('arms-input');
        var armsDisplay = document.getElementById('arms-display');
        if (armsInput && armsDisplay) {
            var updateArms_1 = function () {
                var value = parseInt(armsInput.value);
                if (value < 1)
                    value = 1;
                if (value > 8)
                    value = 8;
                _this.armCount = value;
                armsInput.value = value.toString();
                armsDisplay.textContent = value.toString();
                _this.initGalaxy();
            };
            armsInput.addEventListener('change', updateArms_1);
            armsInput.addEventListener('keyup', function (e) {
                if (e.key === 'Enter')
                    updateArms_1();
            });
        }
        var centerCheck = document.getElementById('center-mass-check');
        if (centerCheck) {
            centerCheck.addEventListener('change', function () {
                _this.hasCenterMass = centerCheck.checked;
                _this.initGalaxy();
            });
        }
        var countSlider = document.getElementById('particle-slider');
        var countDisplay = document.getElementById('count-display');
        if (countSlider && countDisplay) {
            countSlider.addEventListener('input', function (e) {
                _this.N = parseInt(e.target.value);
                countDisplay.textContent = _this.N.toString();
                _this.initGalaxy();
            });
        }
        var dtSlider = document.getElementById('dt-slider');
        var dtDisplay = document.getElementById('dt-display');
        if (dtSlider && dtDisplay) {
            dtSlider.addEventListener('input', function (e) {
                _this.dt = parseFloat(e.target.value);
                dtDisplay.textContent = _this.dt.toFixed(3);
            });
        }
        var thetaSlider = document.getElementById('theta-slider');
        var thetaDisplay = document.getElementById('theta-display');
        if (thetaSlider && thetaDisplay) {
            thetaSlider.addEventListener('input', function (e) {
                _this.theta = parseFloat(e.target.value);
                thetaDisplay.textContent = _this.theta.toFixed(1);
            });
        }
        var resetBtn = document.getElementById('reset-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', function () {
                _this.initGalaxy();
            });
        }
        var pauseBtn = document.getElementById('pause-btn');
        if (pauseBtn) {
            pauseBtn.addEventListener('click', function () {
                _this.paused = !_this.paused;
                pauseBtn.textContent = _this.paused ? 'Продолжить' : 'Пауза';
            });
        }
        var toggleTreeBtn = document.getElementById('toggle-tree-btn');
        if (toggleTreeBtn) {
            toggleTreeBtn.addEventListener('click', function () {
                _this.showTree = !_this.showTree;
                _this.treeLines.visible = _this.showTree;
                toggleTreeBtn.textContent = _this.showTree ? 'Скрыть дерево' : 'Показать дерево';
            });
        }
        var toggleTrailsBtn = document.getElementById('toggle-trails-btn');
        if (toggleTrailsBtn) {
            toggleTrailsBtn.addEventListener('click', function () {
                _this.showTrails = !_this.showTrails;
                _this.trails.visible = _this.showTrails;
                toggleTrailsBtn.textContent = _this.showTrails ? 'Скрыть следы' : 'Следы';
            });
        }
    };
    GalaxySimulation.prototype.onWindowResize = function () {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    };
    GalaxySimulation.prototype.animate = function () {
        var _this = this;
        requestAnimationFrame(function () { return _this.animate(); });
        var frameStart = performance.now();
        if (!this.paused) {
            this.updatePhysics();
        }
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
        var frameTime = performance.now() - frameStart;
        var fps = this.paused ? 0 : Math.round(1000 / frameTime);
        var fpsEl = document.getElementById('fps-counter');
        var frameTimeEl = document.getElementById('frame-time');
        if (fpsEl)
            fpsEl.textContent = fps.toString();
        if (frameTimeEl)
            frameTimeEl.textContent = frameTime.toFixed(1) + 'ms';
    };
    return GalaxySimulation;
}());
window.addEventListener('load', function () {
    new GalaxySimulation();
});
