import {Box3, BufferAttribute, BufferGeometry, LoadingManager, Points, PointsMaterial} from "three";
import URDFLoader from "urdf-loader";
import {degToRad} from "three/src/math/MathUtils.js";

export class RobotRenderer {

    manager = null;
    robot = null;
    loader = null;
    pointsList = []
    pointsMesh = null;
    pointsGeometry = null;

    constructor(urdf_path, onload) {

        const manager = new LoadingManager();
        const loader = new URDFLoader(manager);
        loader.load('SO101/so101_new_calib.urdf', (model) =>{
            this.robot = model;
        });

        manager.onLoad = () => {

            this.robot.rotation.x = -Math.PI / 2;
            this.robot.traverse(c => {
                c.castShadow = true;
            });

            this.robot.updateMatrixWorld(true);

            const bb = new Box3();
            bb.setFromObject(this.robot);
            console.log(bb, this.robot.position);
            onload(this.robot)

        };
    }

    _setupCurve(){
        // Create BufferGeometry
        this.pointsGeometry = new BufferGeometry();
        const positions = new Float32Array(this.pointsList.length * 3);
        this.pointsGeometry .setAttribute('position', new BufferAttribute(positions, 3));

        const material = new PointsMaterial({ color: 0xff0000, size: 0.01 });
        this.pointsMesh = new Points(this.pointsGeometry , material);
        this.pointsMesh.rotation.x = -Math.PI / 2;
        this.robot.parent.add(this.pointsMesh);
    }

    setAction(data) {

        var action = data["action"]
        var is_updated = false;
        for(var joint in action  ){

            var angle = degToRad(action [joint]);
            joint = joint.replace(".pos", "")
            if(this.robot.joints[joint]) {
                is_updated = true
                this.robot.joints[joint].setJointValue(angle);
            }
        }
        this.robot.updateMatrixWorld(true);

        var ee = data['ee'];
        if(ee){
            this.pointsList.push({
                x : ee["ee.x"],
                y : ee["ee.y"],
                z : ee["ee.z"]
            });

            if(this.pointsList.length > 3){
                if(!this.pointsMesh) {
                    this._setupCurve();
                }else{
                    // Make sure BufferAttribute is large enough
                    if (this.pointsGeometry.attributes.position.count !== this.pointsList.length) {
                        const newPositions = new Float32Array(this.pointsList.length * 3);
                        this.pointsGeometry.setAttribute('position', new BufferAttribute(newPositions, 3));
                    }

                    const posAttr = this.pointsGeometry.attributes.position;
                    for (let i = 0; i < this.pointsList.length; i++) {
                        posAttr.setXYZ(i, this.pointsList[i].x, this.pointsList[i].y, this.pointsList[i].z);
                    }
                    posAttr.needsUpdate = true;  // tell Three.js to re-upload to GPU
                }
            }
        }
    }
}