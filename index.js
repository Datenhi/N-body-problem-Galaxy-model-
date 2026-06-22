"use strict";
class Vector3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    add(v) {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }
    sub(v) {
        return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
    }
    mult(s) {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    }
    mag() {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }
    magSq() {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }
    copy() {
        return new Vector3(this.x, this.y, this.z);
    }
}
class Body {
    constructor(pos, vel, mass, isCenter = false) {
        this.pos = pos;
        this.vel = vel;
        this.acc = new Vector3(0, 0, 0);
        this.mass = mass;
        this.isCenter = isCenter;
    }
}
class OctreeNode {
    constructor(center, size) {
        this.center = center;
        this.size = size;
        this.children = null;
        this.body = null;
        this.mass = 0;
        this.com = new Vector3(0, 0, 0);
        this.isLeaf = true;
    }
    getOctant(pos) {
        let index = 0;
        if (pos.x >= this.center.x)
            index += 4;
        if (pos.y >= this.center.y)
            index += 2;
        if (pos.z >= this.center.z)
            index += 1;
        return index;
    }
    insert(body) {
        if (this.mass === 0) {
            this.body = body;
            this.mass = body.mass;
            this.com = body.pos.copy();
            return;
        }
        if (this.isLeaf && this.body !== null) {
            this.subdivide();
            const existingBody = this.body;
            this.body = null;
            this.insertIntoChildren(existingBody);
        }
        this.insertIntoChildren(body);
        this.mass += body.mass;
        this.com.x = (this.com.x * (this.mass - body.mass) + body.pos.x * body.mass) / this.mass;
        this.com.y = (this.com.y * (this.mass - body.mass) + body.pos.y * body.mass) / this.mass;
        this.com.z = (this.com.z * (this.mass - body.mass) + body.pos.z * body.mass) / this.mass;
    }
    insertIntoChildren(body) {
        const octant = this.getOctant(body.pos);
        if (this.children) {
            this.children[octant].insert(body);
        }
    }
    subdivide() {
        this.children = new Array(8);
        const halfSize = this.size / 2;
        const quarterSize = this.size / 4;
        const offsets = [
            [-1, -1, -1], // 0: -X, -Y, -Z
            [-1, -1, 1], // 1: -X, -Y, +Z
            [-1, 1, -1], // 2: -X, +Y, -Z
            [-1, 1, 1], // 3: -X, +Y, +Z
            [1, -1, -1], // 4: +X, -Y, -Z
            [1, -1, 1], // 5: +X, -Y, +Z
            [1, 1, -1], // 6: +X, +Y, -Z
            [1, 1, 1] // 7: +X, +Y, +Z
        ];
        for (let i = 0; i < 8; i++) {
            const dx = offsets[i][0] * quarterSize;
            const dy = offsets[i][1] * quarterSize;
            const dz = offsets[i][2] * quarterSize;
            const childCenter = new Vector3(this.center.x + dx, this.center.y + dy, this.center.z + dz);
            this.children[i] = new OctreeNode(childCenter, halfSize);
        }
        this.isLeaf = false;
    }
    calculateForce(body, theta, G, softening) {
        if (this.mass === 0)
            return new Vector3(0, 0, 0);
        const dx = this.com.x - body.pos.x;
        const dy = this.com.y - body.pos.y;
        const dz = this.com.z - body.pos.z;
        const distSq = dx * dx + dy * dy + dz * dz + softening * softening;
        const dist = Math.sqrt(distSq);
        if (this.isLeaf || (this.size / dist < theta)) {
            const force = G * this.mass / distSq;
            return new Vector3(force * dx / dist, force * dy / dist, force * dz / dist);
        }
        else {
            let totalForce = new Vector3(0, 0, 0);
            if (this.children) {
                for (let child of this.children) {
                    if (child && child.mass > 0) {
                        const f = child.calculateForce(body, theta, G, softening);
                        totalForce = totalForce.add(f);
                    }
                }
            }
            return totalForce;
        }
    }
    countNodes() {
        let count = 1;
        if (this.children) {
            for (let child of this.children) {
                if (child)
                    count += child.countNodes();
            }
        }
        return count;
    }
    getBoundaries() {
        const bounds = [{
                center: this.center,
                size: this.size
            }];
        if (this.children) {
            for (let child of this.children) {
                if (child && child.mass > 0) {
                    bounds.push(...child.getBoundaries());
                }
            }
        }
        return bounds;
    }
}
class ErrorMetrics {
    static calculateForceError(approxForce, exactForce) {
        const diff = approxForce.sub(exactForce);
        const absError = diff.mag();
        const exactMag = exactForce.mag();
        const relError = exactMag > 1e-10
            ? (absError / exactMag) * 100
            : 0;
        return {
            relative: relError,
            absolute: absError
        };
    }
    static calculateAverageError(errors) {
        const n = errors.length;
        if (n === 0)
            return { avgRelative: 0, avgAbsolute: 0, maxRelative: 0, maxAbsolute: 0, stdRelative: 0 };
        const avgRel = errors.reduce((sum, e) => sum + e.relative, 0) / n;
        const avgAbs = errors.reduce((sum, e) => sum + e.absolute, 0) / n;
        const maxRel = Math.max(...errors.map(e => e.relative));
        const maxAbs = Math.max(...errors.map(e => e.absolute));
        const variance = errors.reduce((sum, e) => sum + Math.pow(e.relative - avgRel, 2), 0) / n;
        const stdRel = Math.sqrt(variance);
        return {
            avgRelative: avgRel,
            avgAbsolute: avgAbs,
            maxRelative: maxRel,
            maxAbsolute: maxAbs,
            stdRelative: stdRel
        };
    }
}
class GalaxySimulation {
    constructor() {
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
        this.enableErrorCalc = false;
        this.errorSampleIndices = [];
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
    initThree() {
        this.scene = new THREE.Scene();
        this.scene.fog = new THREE.FogExp2(0x000000, 0.02);
        this.camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
        this.camera.position.set(0, 20, 40);
        this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        const container = document.getElementById('canvas-container');
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
            color: 0xff00ff,
            transparent: true,
            opacity: 0.6,
            blending: THREE.AdditiveBlending
        }));
        this.scene.add(this.trails);
        this.trails.visible = false;
        window.addEventListener('resize', () => this.onWindowResize());
    }
    initGalaxy() {
        this.bodies = [];
        this.errorSampleIndices = [];
        const armSpread = 0.3;
        if (this.hasCenterMass) {
            const centerBody = new Body(new Vector3(0, 0, 0), new Vector3(0, 0, 0), this.centerMass, true);
            this.bodies.push(centerBody);
        }
        for (let i = 0; i < this.N; i++) {
            const armIndex = i % this.armCount;
            const armOffset = (armIndex / this.armCount) * Math.PI * 2;
            const dist = Math.random() * 15 + 2;
            const angle = armOffset + dist * 0.3 + (Math.random() - 0.5) * armSpread;
            const x = Math.cos(angle) * dist;
            const z = Math.sin(angle) * dist;
            const y = (Math.random() - 0.5) * 2 * Math.exp(-dist / 10);
            const totalMass = this.hasCenterMass ? this.centerMass + this.N * 3 : this.N * 3;
            const v = Math.sqrt(this.G * totalMass / dist) * 0.8;
            const vx = -Math.sin(angle) * v;
            const vz = Math.cos(angle) * v;
            const pos = new Vector3(x, y, z);
            const vel = new Vector3(vx, (Math.random() - 0.5) * 0.1, vz);
            const mass = 1.0 + Math.random() * 2.0;
            this.bodies.push(new Body(pos, vel, mass));
        }
        this.buildOctree();
        for (let body of this.bodies) {
            if (this.root) {
                body.acc = this.root.calculateForce(body, this.theta, this.G, this.softening);
            }
        }
        this.positions.fill(0);
        this.colors.fill(0);
        this.sizes.fill(0);
        this.trailHistory = [];
        for (let i = 0; i < this.N; i++) {
            this.trailHistory[i] = [];
        }
        this.geometry.setDrawRange(0, this.N);
        this.updateGeometry();
    }
    buildOctree() {
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;
        for (let body of this.bodies) {
            minX = Math.min(minX, body.pos.x);
            maxX = Math.max(maxX, body.pos.x);
            minY = Math.min(minY, body.pos.y);
            maxY = Math.max(maxY, body.pos.y);
            minZ = Math.min(minZ, body.pos.z);
            maxZ = Math.max(maxZ, body.pos.z);
        }
        const center = new Vector3((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);
        const size = Math.max(maxX - minX, maxY - minY, maxZ - minZ) * 1.1;
        this.root = new OctreeNode(center, size);
        for (let body of this.bodies) {
            this.root.insert(body);
        }
        return this.root.countNodes();
    }
    calculateExactForce(body, allBodies) {
        let totalForce = new Vector3(0, 0, 0);
        for (let other of allBodies) {
            if (other === body)
                continue;
            const dx = other.pos.x - body.pos.x;
            const dy = other.pos.y - body.pos.y;
            const dz = other.pos.z - body.pos.z;
            const distSq = dx * dx + dy * dy + dz * dz + this.softening * this.softening;
            const dist = Math.sqrt(distSq);
            const force = this.G * other.mass / distSq;
            totalForce = totalForce.add(new Vector3(force * dx / dist, force * dy / dist, force * dz / dist));
        }
        return totalForce;
    }
    calculateError() {
        const sampleIndices = [];
        const startIndex = this.hasCenterMass ? 1 : 0;
        const errorSampleSize = this.N * 0.15;
        const step = (this.N - startIndex) / errorSampleSize;
        for (let i = 0; i < errorSampleSize; i++) {
            const idx = startIndex + Math.round(i * step);
            if (idx < this.bodies.length) {
                sampleIndices.push(idx);
            }
        }
        this.errorSampleIndices = sampleIndices;
        const approxForces = [];
        for (let idx of sampleIndices) {
            if (this.root) {
                approxForces.push(this.root.calculateForce(this.bodies[idx], this.theta, this.G, this.softening));
            }
        }
        const exactForces = [];
        for (let idx of sampleIndices) {
            exactForces.push(this.calculateExactForce(this.bodies[idx], this.bodies));
        }
        const errors = [];
        for (let i = 0; i < sampleIndices.length; i++) {
            errors.push(ErrorMetrics.calculateForceError(approxForces[i], exactForces[i]));
        }
        const metrics = ErrorMetrics.calculateAverageError(errors);
        return {
            avgRelative: metrics.avgRelative,
            maxRelative: metrics.maxRelative,
            stdRelative: metrics.stdRelative,
            sampleSize: sampleIndices.length
        };
    }
    updatePhysics() {
        const calcStart = performance.now();
        const nodeCount = this.buildOctree();
        const nodeCountEl = document.getElementById('node-count');
        if (nodeCountEl)
            nodeCountEl.textContent = nodeCount.toString();
        for (let body of this.bodies) {
            if (this.root) {
                body.acc = this.root.calculateForce(body, this.theta, this.G, this.softening);
            }
        }
        if (this.enableErrorCalc && Math.random() < 0.2) {
            const errorResult = this.calculateError();
            if (errorResult) {
                this.updateErrorDisplay(errorResult);
            }
        }
        else if (!this.enableErrorCalc) {
            this.errorSampleIndices = [];
        }
        for (let body of this.bodies) {
            if (body.isCenter) {
                body.vel = new Vector3(0, 0, 0);
                body.acc = new Vector3(0, 0, 0);
                continue;
            }
            const halfStepVel = body.vel.add(body.acc.mult(this.dt / 2));
            body.pos = body.pos.add(halfStepVel.mult(this.dt));
            body.vel = halfStepVel;
        }
        this.buildOctree();
        for (let body of this.bodies) {
            if (body.isCenter)
                continue;
            if (this.root) {
                body.acc = this.root.calculateForce(body, this.theta, this.G, this.softening);
            }
        }
        let totalEnergy = 0;
        for (let body of this.bodies) {
            if (body.isCenter)
                continue;
            body.vel = body.vel.add(body.acc.mult(this.dt / 2));
            const vSq = body.vel.magSq();
            totalEnergy += 0.5 * body.mass * vSq;
        }
        const calcTime = performance.now() - calcStart;
        const calcTimeEl = document.getElementById('calc-time');
        if (calcTimeEl)
            calcTimeEl.textContent = calcTime.toFixed(1) + 'ms';
        const energyEl = document.getElementById('energy');
        if (energyEl)
            energyEl.textContent = totalEnergy.toFixed(2);
        this.updateGeometry();
        if (this.showTree)
            this.updateTreeVisualization();
        if (this.showTrails)
            this.updateTrails();
    }
    updateErrorDisplay(metrics) {
        const avgEl = document.getElementById('error-avg');
        const maxEl = document.getElementById('error-max');
        const stdEl = document.getElementById('error-std');
        const speedupEl = document.getElementById('error-speedup');
        if (avgEl)
            avgEl.textContent = metrics.avgRelative.toFixed(2) + '%';
        if (maxEl)
            maxEl.textContent = metrics.maxRelative.toFixed(2) + '%';
        if (stdEl)
            stdEl.textContent = metrics.stdRelative.toFixed(2) + '%';
    }
    updateGeometry() {
        const startIndex = this.hasCenterMass ? 1 : 0;
        const visualN = this.N;
        const positions = this.geometry.attributes.position.array;
        const colors = this.geometry.attributes.color.array;
        const sizes = this.geometry.attributes.size.array;
        const errorIndicesMap = {};
        for (let idx of this.errorSampleIndices) {
            const visualIdx = idx - startIndex;
            if (visualIdx >= 0 && visualIdx < visualN) {
                errorIndicesMap[visualIdx] = true;
            }
        }
        for (let i = 0; i < visualN; i++) {
            const bodyIndex = i + startIndex;
            const body = this.bodies[bodyIndex];
            positions[i * 3] = body.pos.x;
            positions[i * 3 + 1] = body.pos.y;
            positions[i * 3 + 2] = body.pos.z;
            const isErrorStar = this.enableErrorCalc && errorIndicesMap[i] === true;
            if (isErrorStar) {
                colors[i * 3] = 1.0;
                colors[i * 3 + 1] = 0.2;
                colors[i * 3 + 2] = 0.3;
                sizes[i] = 0.8;
            }
            else {
                const speed = body.vel.mag();
                const t = Math.min(speed / 2, 1);
                colors[i * 3] = 0.3 + t * 0.7;
                colors[i * 3 + 1] = 0.5 + t * 0.3;
                colors[i * 3 + 2] = 1.0 - t * 0.5;
                sizes[i] = 0.3 + body.mass * 0.2;
            }
        }
        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.attributes.color.needsUpdate = true;
        this.geometry.attributes.size.needsUpdate = true;
    }
    updateTreeVisualization() {
        if (!this.root)
            return;
        const bounds = this.root.getBoundaries();
        const vertices = [];
        for (let b of bounds) {
            const hs = b.size / 2;
            const cx = b.center.x;
            const cy = b.center.y;
            const cz = b.center.z;
            const corners = [
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
            for (let c of corners) {
                vertices.push(...c);
            }
        }
        this.treeLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    }
    updateTrails() {
        const startIndex = this.hasCenterMass ? 1 : 0;
        for (let i = 0; i < this.N; i++) {
            const bodyIndex = i + startIndex;
            const body = this.bodies[bodyIndex];
            this.trailHistory[i].push(body.pos.copy());
            if (this.trailHistory[i].length > 20) {
                this.trailHistory[i].shift();
            }
        }
        const vertices = [];
        for (let i = 0; i < this.N; i++) {
            const history = this.trailHistory[i];
            for (let j = 0; j < history.length - 1; j++) {
                vertices.push(history[j].x, history[j].y, history[j].z, history[j + 1].x, history[j + 1].y, history[j + 1].z);
            }
        }
        this.trails.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    }
    setupUI() {
        const armsInput = document.getElementById('arms-input');
        const armsDisplay = document.getElementById('arms-display');
        if (armsInput && armsDisplay) {
            const updateArms = () => {
                let value = parseInt(armsInput.value);
                if (value < 1)
                    value = 1;
                if (value > 8)
                    value = 8;
                this.armCount = value;
                armsInput.value = value.toString();
                armsDisplay.textContent = value.toString();
                this.initGalaxy();
            };
            armsInput.addEventListener('change', updateArms);
            armsInput.addEventListener('keyup', (e) => {
                if (e.key === 'Enter')
                    updateArms();
            });
        }
        const centerCheck = document.getElementById('center-mass-check');
        if (centerCheck) {
            centerCheck.addEventListener('change', () => {
                this.hasCenterMass = centerCheck.checked;
                this.initGalaxy();
            });
        }
        const countSlider = document.getElementById('particle-slider');
        const countDisplay = document.getElementById('count-display');
        if (countSlider && countDisplay) {
            countSlider.addEventListener('input', (e) => {
                this.N = parseInt(e.target.value);
                countDisplay.textContent = this.N.toString();
                this.initGalaxy();
            });
        }
        const dtSlider = document.getElementById('dt-slider');
        const dtDisplay = document.getElementById('dt-display');
        if (dtSlider && dtDisplay) {
            dtSlider.addEventListener('input', (e) => {
                this.dt = parseFloat(e.target.value);
                dtDisplay.textContent = this.dt.toFixed(3);
            });
        }
        const thetaSlider = document.getElementById('theta-slider');
        const thetaDisplay = document.getElementById('theta-display');
        if (thetaSlider && thetaDisplay) {
            thetaSlider.addEventListener('input', (e) => {
                this.theta = parseFloat(e.target.value);
                thetaDisplay.textContent = this.theta.toFixed(1);
            });
        }
        const errorCheck = document.getElementById('error-check');
        if (errorCheck) {
            errorCheck.addEventListener('change', () => {
                this.enableErrorCalc = errorCheck.checked;
                if (!errorCheck.checked) {
                    this.errorSampleIndices = [];
                    this.resetErrorDisplay();
                }
            });
        }
        const resetBtn = document.getElementById('reset-btn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                this.initGalaxy();
            });
        }
        const pauseBtn = document.getElementById('pause-btn');
        if (pauseBtn) {
            pauseBtn.addEventListener('click', () => {
                this.paused = !this.paused;
                pauseBtn.textContent = this.paused ? 'Продолжить' : 'Пауза';
            });
        }
        const toggleTreeBtn = document.getElementById('toggle-tree-btn');
        if (toggleTreeBtn) {
            toggleTreeBtn.addEventListener('click', () => {
                this.showTree = !this.showTree;
                this.treeLines.visible = this.showTree;
                toggleTreeBtn.textContent = this.showTree ? 'Скрыть дерево' : 'Показать дерево';
            });
        }
        const toggleTrailsBtn = document.getElementById('toggle-trails-btn');
        if (toggleTrailsBtn) {
            toggleTrailsBtn.addEventListener('click', () => {
                this.showTrails = !this.showTrails;
                this.trails.visible = this.showTrails;
                toggleTrailsBtn.textContent = this.showTrails ? 'Скрыть следы' : 'Следы';
            });
        }
    }
    resetErrorDisplay() {
        const avgEl = document.getElementById('error-avg');
        const maxEl = document.getElementById('error-max');
        const stdEl = document.getElementById('error-std');
        if (avgEl)
            avgEl.textContent = '—';
        if (maxEl)
            maxEl.textContent = '—';
        if (stdEl)
            stdEl.textContent = '—';
    }
    onWindowResize() {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }
    animate() {
        requestAnimationFrame(() => this.animate());
        const frameStart = performance.now();
        if (!this.paused) {
            this.updatePhysics();
        }
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
        const frameTime = performance.now() - frameStart;
        const fps = this.paused ? 0 : Math.round(1000 / frameTime);
        const fpsEl = document.getElementById('fps-counter');
        const frameTimeEl = document.getElementById('frame-time');
        if (fpsEl)
            fpsEl.textContent = fps.toString();
        if (frameTimeEl)
            frameTimeEl.textContent = frameTime.toFixed(1) + 'ms';
    }
}
window.addEventListener('load', () => {
    new GalaxySimulation();
});
