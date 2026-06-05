declare const THREE: any;

interface Vector3Like {
    x: number;
    y: number;
    z: number;
}

class Vector3 implements Vector3Like {
    x: number;
    y: number;
    z: number;

    constructor(x: number = 0, y: number = 0, z: number = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    add(v: Vector3): Vector3 {
        return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
    }

    mult(s: number): Vector3 {
        return new Vector3(this.x * s, this.y * s, this.z * s);
    }

    mag(): number {
        return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    magSq(): number {
        return this.x * this.x + this.y * this.y + this.z * this.z;
    }

    copy(): Vector3 {
        return new Vector3(this.x, this.y, this.z);
    }
}

class Body {
    pos: Vector3;
    vel: Vector3;
    acc: Vector3;
    mass: number;
    isCenter: boolean;

    constructor(pos: Vector3, vel: Vector3, mass: number, isCenter =false) {
        this.pos = pos;
        this.vel = vel;
        this.acc = new Vector3(0, 0, 0);
        this.mass = mass;
        this.isCenter = isCenter;
    }
}

class OctreeNode {
    center: Vector3;
    size: number;
    children: OctreeNode[] | null;
    body: Body | null;
    mass: number;
    com: Vector3;
    isLeaf: boolean;

    constructor(center: Vector3, size: number) {
        this.center = center;
        this.size = size;
        this.children = null;
        this.body = null;
        this.mass = 0;
        this.com = new Vector3(0, 0, 0);
        this.isLeaf = true;
    }

    getOctant(pos: Vector3): number {
        let index = 0;
        if (pos.x >= this.center.x) index += 4;
        if (pos.y >= this.center.y) index += 2;
        if (pos.z >= this.center.z) index += 1;
        return index;
    }

    insert(body: Body): void {
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

    insertIntoChildren(body: Body): void {
        const octant = this.getOctant(body.pos);
        if (this.children) {
            this.children[octant].insert(body);
        }
    }

    subdivide(): void {
        this.children = new Array(8);
        const halfSize = this.size / 2;
        const quarterSize = this.size / 4;

        const offsets = [
            [-1, -1, -1],  // 0: -X, -Y, -Z
            [-1, -1,  1],  // 1: -X, -Y, +Z
            [-1,  1, -1],  // 2: -X, +Y, -Z
            [-1,  1,  1],  // 3: -X, +Y, +Z
            [ 1, -1, -1],  // 4: +X, -Y, -Z
            [ 1, -1,  1],  // 5: +X, -Y, +Z
            [ 1,  1, -1],  // 6: +X, +Y, -Z
            [ 1,  1,  1]   // 7: +X, +Y, +Z
        ];

        for (let i = 0; i < 8; i++) {
            const dx = offsets[i][0] * quarterSize;
            const dy = offsets[i][1] * quarterSize;
            const dz = offsets[i][2] * quarterSize;

            const childCenter = new Vector3(
                this.center.x + dx,
                this.center.y + dy,
                this.center.z + dz
            );

            this.children[i] = new OctreeNode(childCenter, halfSize);
        }

        this.isLeaf = false;
    }

    calculateForce(body: Body, theta: number, G: number, softening: number): Vector3 {
        if (this.mass === 0) return new Vector3(0, 0, 0);

        const dx = this.com.x - body.pos.x;
        const dy = this.com.y - body.pos.y;
        const dz = this.com.z - body.pos.z;
        const distSq = dx * dx + dy * dy + dz * dz + softening * softening;
        const dist = Math.sqrt(distSq);

        if (this.isLeaf || (this.size / dist < theta)) {
            const force = G * this.mass / distSq;
            return new Vector3(
                force * dx / dist,
                force * dy / dist,
                force * dz / dist
            );
        } else {
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

    countNodes(): number {
        let count = 1;
        if (this.children) {
            for (let child of this.children) {
                if (child) count += child.countNodes();
            }
        }
        return count;
    }

    getBoundaries(): Array<{center: Vector3, size: number}> {
        const bounds: Array<{center: Vector3, size: number}> = [{
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

class GalaxySimulation {
    private bodies: Body[] = [];
    private N: number = 2000;
    private theta: number = 0.5;
    private dt: number = 0.005;
    private G: number = 0.1;
    private softening: number = 0.01;
    private paused: boolean = false;
    private showTree: boolean = false;
    private showTrails: boolean = false;
    private root: OctreeNode | null = null;

    private armCount: number = 3;
    private hasCenterMass: boolean = true;
    private centerMass: number = 10000;

    private scene: any;
    private camera: any;
    private renderer: any;
    private controls: any;
    private points: any;
    private treeLines: any;
    private trails: any;
    private geometry: any;
    private trailGeometry: any;
    private material: any;
    private centerSphere: any;

    private positions: Float32Array;
    private colors: Float32Array;
    private sizes: Float32Array;
    private trailPositions: Float32Array;
    private trailHistory: Vector3[][];

    constructor() {
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

    private initThree(): void {
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

        this.treeLines = new THREE.LineSegments(
            new THREE.BufferGeometry(),
            new THREE.LineBasicMaterial({ color: 0x4f46e5, transparent: true, opacity: 0.3 })
        );
        this.scene.add(this.treeLines);

        this.trailGeometry = new THREE.BufferGeometry();
        this.trailGeometry.setAttribute('position', new THREE.BufferAttribute(this.trailPositions, 3));
        this.trails = new THREE.LineSegments(
            this.trailGeometry,
            new THREE.LineBasicMaterial({
                color: 0x6366f1,
                transparent: true,
                opacity: 0.2,
                blending: THREE.AdditiveBlending
            })
        );
        this.scene.add(this.trails);
        this.trails.visible = false;

        window.addEventListener('resize', () => this.onWindowResize());
    }

    private initGalaxy(): void {
        this.bodies = [];
        const armSpread = 0.3;

        if (this.hasCenterMass) {
            const centerBody = new Body(
                new Vector3(0, 0, 0),      // Позиция в центре
                new Vector3(0, 0, 0),      // Нулевая скорость (неподвижна)
                this.centerMass,
                true                       // Флаг isCenter
            );
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

            const totalMass = this.hasCenterMass ? this.centerMass : 1000;
            const v = Math.sqrt(this.G * totalMass / dist) * 0.8;
            const vx = -Math.sin(angle) * v;
            const vz = Math.cos(angle) * v;

            const pos = new Vector3(x, y, z);
            const vel = new Vector3(vx, (Math.random() - 0.5) * 0.1, vz);
            const mass = 1.0 + Math.random() * 2.0;

            this.bodies.push(new Body(pos, vel, mass));
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

    private buildOctree(): number {
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

        const center = new Vector3(
            (minX + maxX) / 2,
            (minY + maxY) / 2,
            (minZ + maxZ) / 2
        );
        const size = Math.max(maxX - minX, maxY - minY, maxZ - minZ) * 1.1;

        this.root = new OctreeNode(center, size);

        for (let body of this.bodies) {
            this.root.insert(body);
        }

        return this.root.countNodes();
    }

    private updatePhysics(): void {
        const calcStart = performance.now();

        const nodeCount = this.buildOctree();
        const nodeCountEl = document.getElementById('node-count');
        if (nodeCountEl) nodeCountEl.textContent = nodeCount.toString();

        for (let body of this.bodies) {
            if (this.root) {
                body.acc = this.root.calculateForce(body, this.theta, this.G, this.softening);
            }
        }

        const calcTime = performance.now() - calcStart;
        const calcTimeEl = document.getElementById('calc-time');
        if (calcTimeEl) calcTimeEl.textContent = calcTime.toFixed(1) + 'ms';

        let totalEnergy = 0;
        let totalMass = 0;

        for (let body of this.bodies) {
            if (body.isCenter) {
                body.vel = new Vector3(0, 0, 0);
                body.acc = new Vector3(0, 0, 0);
                totalMass += body.mass;
                continue;
            }
            body.vel = body.vel.add(body.acc.mult(this.dt));
            body.pos = body.pos.add(body.vel.mult(this.dt));

            const vSq = body.vel.magSq();
            totalEnergy += 0.5 * body.mass * vSq;
            totalMass += body.mass;
        }

        if (this.centerSphere) {
            this.centerSphere.visible = this.hasCenterMass;
        }

        const energyEl = document.getElementById('energy');
        if (energyEl) energyEl.textContent = totalEnergy.toFixed(2);

        this.updateGeometry();
        if (this.showTree) this.updateTreeVisualization();
        if (this.showTrails) this.updateTrails();
    }

    private updateGeometry(): void {
        const startIndex = this.hasCenterMass ? 1 : 0;
        const visualN = this.N;

        const positions = this.geometry.attributes.position.array as Float32Array;
        const colors = this.geometry.attributes.color.array as Float32Array;
        const sizes = this.geometry.attributes.size.array as Float32Array;

        for (let i = 0; i < visualN; i++) {
            const bodyIndex = i + startIndex;
            const body = this.bodies[bodyIndex];
            positions[i * 3] = body.pos.x;
            positions[i * 3 + 1] = body.pos.y;
            positions[i * 3 + 2] = body.pos.z;

            const speed = body.vel.mag();
            const t = Math.min(speed / 2, 1);
            colors[i * 3] = 0.3 + t * 0.7;
            colors[i * 3 + 1] = 0.5 + t * 0.3;
            colors[i * 3 + 2] = 1.0 - t * 0.5;

            sizes[i] = 0.3 + body.mass * 0.2;
        }

        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.attributes.color.needsUpdate = true;
        this.geometry.attributes.size.needsUpdate = true;
    }

    private updateTreeVisualization(): void {
        if (!this.root) return;

        const bounds = this.root.getBoundaries();
        const vertices: number[] = [];

        for (let b of bounds) {
            const hs = b.size / 2;
            const cx = b.center.x;
            const cy = b.center.y;
            const cz = b.center.z;

            const corners = [
                [cx-hs, cy-hs, cz-hs], [cx+hs, cy-hs, cz-hs],
                [cx+hs, cy-hs, cz-hs], [cx+hs, cy+hs, cz-hs],
                [cx+hs, cy+hs, cz-hs], [cx-hs, cy+hs, cz-hs],
                [cx-hs, cy+hs, cz-hs], [cx-hs, cy-hs, cz-hs],
                [cx-hs, cy-hs, cz+hs], [cx+hs, cy-hs, cz+hs],
                [cx+hs, cy-hs, cz+hs], [cx+hs, cy+hs, cz+hs],
                [cx+hs, cy+hs, cz+hs], [cx-hs, cy+hs, cz+hs],
                [cx-hs, cy+hs, cz+hs], [cx-hs, cy-hs, cz+hs],
                [cx-hs, cy-hs, cz-hs], [cx-hs, cy-hs, cz+hs],
                [cx+hs, cy-hs, cz-hs], [cx+hs, cy-hs, cz+hs],
                [cx+hs, cy+hs, cz-hs], [cx+hs, cy+hs, cz+hs],
                [cx-hs, cy+hs, cz-hs], [cx-hs, cy+hs, cz+hs]
            ];

            for (let c of corners) {
                vertices.push(...c);
            }
        }

        this.treeLines.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    }

    private updateTrails(): void {
        for (let i = 0; i < this.N; i++) {
            const body = this.bodies[i];
            this.trailHistory[i].push(body.pos.copy());
            if (this.trailHistory[i].length > 20) {
                this.trailHistory[i].shift();
            }
        }

        const vertices: number[] = [];
        for (let i = 0; i < this.N; i++) {
            const history = this.trailHistory[i];
            for (let j = 0; j < history.length - 1; j++) {
                vertices.push(
                    history[j].x, history[j].y, history[j].z,
                    history[j+1].x, history[j+1].y, history[j+1].z
                );
            }
        }

        this.trails.geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    }

    private setupUI(): void {
        const armsInput = document.getElementById('arms-input') as HTMLInputElement;
        const armsDisplay = document.getElementById('arms-display');

        if (armsInput && armsDisplay) {
            const updateArms = () => {
                let value = parseInt(armsInput.value);
                if (value < 1) value = 1;
                if (value > 8) value = 8;

                this.armCount = value;
                armsInput.value = value.toString();
                armsDisplay.textContent = value.toString();

                this.initGalaxy();
            };

            armsInput.addEventListener('change', updateArms);
            armsInput.addEventListener('keyup', (e) => {
                if (e.key === 'Enter') updateArms();
            });
        }

        const centerCheck = document.getElementById('center-mass-check') as HTMLInputElement;
        if (centerCheck) {
            centerCheck.addEventListener('change', () => {
                this.hasCenterMass = centerCheck.checked;
                this.initGalaxy();
            });
        }

        const countSlider = document.getElementById('particle-slider') as HTMLInputElement;
        const countDisplay = document.getElementById('count-display');
        if (countSlider && countDisplay) {
            countSlider.addEventListener('input', (e) => {
                this.N = parseInt((e.target as HTMLInputElement).value);
                countDisplay.textContent = this.N.toString();
                this.initGalaxy();
            });
        }

        const dtSlider = document.getElementById('dt-slider') as HTMLInputElement;
        const dtDisplay = document.getElementById('dt-display');
        if (dtSlider && dtDisplay) {
            dtSlider.addEventListener('input', (e) => {
                this.dt = parseFloat((e.target as HTMLInputElement).value);
                dtDisplay.textContent = this.dt.toFixed(3);
            });
        }

        const thetaSlider = document.getElementById('theta-slider') as HTMLInputElement;
        const thetaDisplay = document.getElementById('theta-display');
        if (thetaSlider && thetaDisplay) {
            thetaSlider.addEventListener('input', (e) => {
                this.theta = parseFloat((e.target as HTMLInputElement).value);
                thetaDisplay.textContent = this.theta.toFixed(1);
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

    private onWindowResize(): void {
        this.camera.aspect = window.innerWidth / window.innerHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
    }

    private animate(): void {
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
        if (fpsEl) fpsEl.textContent = fps.toString();
        if (frameTimeEl) frameTimeEl.textContent = frameTime.toFixed(1) + 'ms';
    }
}

window.addEventListener('load', () => {
    new GalaxySimulation();
});
