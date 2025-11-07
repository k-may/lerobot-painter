import './style.css'
import {
    AmbientLight, Box3,
    Color,
    DirectionalLight,
    LoadingManager,
    Mesh, PCFSoftShadowMap,
    PerspectiveCamera, PlaneGeometry,
    Scene, ShadowMaterial,
    WebGLRenderer
} from 'three';
import URDFLoader from 'urdf-loader';
import {OrbitControls} from "three/addons";

let scene, camera, renderer, robot, controls;

scene = new Scene();
scene.background = new Color(0x263238);

camera = new PerspectiveCamera();
camera.position.set(0.3,0.3, 0.3);
camera.lookAt(0, 0, 0);

renderer = new WebGLRenderer({ antialias: true });
// renderer.outputEncoding = THREE.sRGBEncoding;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);

const directionalLight = new DirectionalLight(0xffffff, 1.0);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.setScalar(1024);
directionalLight.position.set(5, 30, 5);
scene.add(directionalLight);

const ambientLight = new AmbientLight(0xffffff, 0.2);
scene.add(ambientLight);

const ground = new Mesh(new PlaneGeometry(), new ShadowMaterial({ opacity: 0.25 }));
ground.rotation.x = -Math.PI / 2;
ground.scale.setScalar(30);
ground.receiveShadow = true;
scene.add(ground);

controls = new OrbitControls(camera, renderer.domElement);
controls.minDistance = 1;
controls.target.y = 0.2;
controls.update();

const manager = new LoadingManager();
const loader = new URDFLoader( manager );
loader.load( 'SO101/so101_new_calib.urdf', function ( model ) {
    scene.add( model );
    robot = model;
} );
manager.onLoad = () => {

    robot.rotation.x = -Math.PI / 2;
    robot.traverse(c => {
        c.castShadow = true;
    });

    robot.updateMatrixWorld(true);

    const bb = new Box3();
    bb.setFromObject(robot);
    console.log(bb, robot.position);
    // robot.position.y -= bb.min.y;
    scene.add(robot);

};
function onResize() {

    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);

    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
}

onResize();
window.addEventListener('resize', onResize);

function loop() {

    renderer.render(scene, camera);
    window.requestAnimationFrame(loop)
}

loop();

const ws = new WebSocket("ws://localhost:8765");

ws.onopen = () => console.log("Connected to WS server");
ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log("Joint angles:", data);
};