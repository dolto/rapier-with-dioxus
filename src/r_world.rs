use std::collections::{HashMap, HashSet};

use dioxus::prelude::*;
use rapier2d::prelude::*;

pub struct RWorld {
    pub rigid_body_set: RigidBodySet,
    pub collider_set: ColliderSet,

    physics_pipeline: PhysicsPipeline,

    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    islands: IslandManager,
    broad_phase: Box<dyn BroadPhase>,
    narrow_phase: NarrowPhase,
    pub impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    hooks: Box<dyn PhysicsHooks>,
    events: Box<dyn rapier2d::pipeline::EventHandler>,
    shapes: HashSet<(ColliderHandle, RigidBodyHandle)>,
    elements: HashMap<(ColliderHandle, RigidBodyHandle), Element>,
}

impl RWorld {
    pub fn new(gx: f32, gy: f32) -> Self {
        RWorld {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            physics_pipeline: PhysicsPipeline::new(),
            gravity: vector![gx, gy],
            integration_parameters: IntegrationParameters::default(),
            islands: IslandManager::new(),
            broad_phase: Box::new(DefaultBroadPhase::new()),
            narrow_phase: NarrowPhase::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            hooks: Box::new(()),
            events: Box::new(()),
            shapes: HashSet::new(),
            elements: HashMap::new(),
        }
    }

    pub fn step(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.islands,
            &mut *self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &*self.hooks,
            &*self.events,
        )
    }

    pub fn set_gravity(&mut self, gx: f32, gy: f32) {
        self.gravity = vector![gx, gy];
    }

    pub fn get_gravity(&self) -> (f32, f32) {
        (*self.gravity.get(0).unwrap(), *self.gravity.get(1).unwrap())
    }

    pub fn insert_rigi_collider_shape(
        &mut self,
        rigid_body: RigidBody,
        collider: Collider,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let rigid_handle = self.rigid_body_set.insert(rigid_body);
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rigid_handle, &mut self.rigid_body_set);

        self.shapes
            .insert((collider_handle.clone(), rigid_handle.clone()));

        (rigid_handle, collider_handle)
    }
    pub fn insert_rigi_collider(
        &mut self,
        rigid_body: RigidBody,
        collider: Collider,
    ) -> (RigidBodyHandle, ColliderHandle) {
        let rigid_handle = self.rigid_body_set.insert(rigid_body);
        let collider_handle =
            self.collider_set
                .insert_with_parent(collider, rigid_handle, &mut self.rigid_body_set);

        (rigid_handle, collider_handle)
    }

    pub fn insert_rigi_collider_element(
        &mut self,
        rigid_handle: RigidBodyHandle,
        collider_handle: ColliderHandle,
        element: Element
    ) {
        self.shapes.remove(&(collider_handle.clone(), rigid_handle.clone()));
        self.elements.insert((collider_handle, rigid_handle), element);
    }
    // 요소에 이벤트를 넣기 위해선, 콜백이 아니라 직접 구현해야함
    // pub fn insert_rigi_collider_element(
    //     &mut self,
    //     rigid_body: RigidBody,
    //     shape: SharedShape,
    //     element: Element,
    // ) -> (RigidBodyHandle, ColliderHandle) {
    //     let collider = ColliderBuilder::new(shape.clone());
    //     let rigid_handle = self.rigid_body_set.insert(rigid_body);
    //     let collider_handle =
    //         self.collider_set
    //             .insert_with_parent(collider, rigid_handle, &mut self.rigid_body_set);

    //     self.elements
    //         .insert((collider_handle.clone(), rigid_handle.clone()), element);

    //     (rigid_handle, collider_handle)
    // }

    pub fn remove_rigi_collider(&mut self, ch: ColliderHandle) {
        let rigihandle = self.collider_set.get(ch).unwrap().parent().unwrap();
        self.elements.remove(&(ch.clone(), rigihandle.clone()));
        self.shapes.remove(&(ch.clone(), rigihandle.clone()));
        self.rigid_body_set.remove(
            rigihandle,
            &mut self.islands,
            &mut self.collider_set,
            &mut self.impulse_joints,
            &mut self.multibody_joints,
            true,
        );
    }

    pub fn insert_fixed_joint(&mut self, rigi1: RigidBodyHandle, xy1: (f32, f32) ,rigi2: RigidBodyHandle, xy2: (f32,f32)) -> ImpulseJointHandle{
        // 거리와 회전이 고정된 조인트
        let joint = FixedJointBuilder::new().local_anchor1(point![xy1.0, xy1.1]).local_anchor2(point![xy2.0,xy2.1]);
        self.impulse_joints.insert(rigi1, rigi2, joint, true)
    }
    pub fn insert_revolute_joint(&mut self, rigi1: RigidBodyHandle, xy1: (f32, f32) ,rigi2: RigidBodyHandle, xy2: (f32,f32)) -> ImpulseJointHandle{
        // 거리만 유지하는 조인트 회전은 자유롭다
        let joint = RevoluteJointBuilder::new().local_anchor1(point![xy1.0, xy1.1]).local_anchor2(point![xy2.0,xy2.1]);
        self.impulse_joints.insert(rigi1, rigi2, joint, true)
    }
    pub fn insert_rope_joint(&mut self, rigi1: RigidBodyHandle, xy1: (f32, f32) ,rigi2: RigidBodyHandle, xy2: (f32,f32), distence: f32) -> ImpulseJointHandle{
        // 거리만 유지하는 조인트 회전은 자유롭다
        let joint = RopeJointBuilder::new(distence).local_anchor1(point![xy1.0, xy1.1]).local_anchor2(point![xy2.0,xy2.1]);
        self.impulse_joints.insert(rigi1, rigi2, joint, true)
    }
    pub fn insert_prismatic_joint(&mut self, axis:UnitVector<Real>,rigi1: RigidBodyHandle, xy1: (f32, f32) ,rigi2: RigidBodyHandle, xy2: (f32,f32), limit: [f32;2]) -> ImpulseJointHandle{
        // axis축으로만 이동이 가능하며, 최대 limits거리를 벗어나지 않는다.
        // axis: Vector::x_axis(); 등과 같이 설정 가능
        let joint = PrismaticJointBuilder::new(axis).local_anchor1(point![xy1.0, xy1.1]).local_anchor2(point![xy2.0,xy2.1]).limits(limit);
        self.impulse_joints.insert(rigi1, rigi2, joint, true)
    }

    pub fn render(&self) -> Element {
        let shapes = self.shapes.iter().map(|(ch, rh)| {
            let rigid = self.rigid_body_set.get(*rh);
            if let Some(rigid) = rigid {
                if let Some(collider) = self.collider_set.get(*ch){
                    let shape = collider.shape();
                    let (x, y) = (
                        rigid.position().translation.x,
                        rigid.position().translation.y,
                    );
                    let rotate = rigid.rotation().angle();
                    let style;
                    let mut width= 0.;
                    let mut height= 0.;
                    if let Some(ball) = shape.as_ball() {
                        width = ball.radius * 2.;
                        height = ball.radius * 2.;
                        style = format!("border-radius: 100%;");
                    } else if let Some(cuboid) = shape.as_cuboid() {
                        width = cuboid.half_extents.x * 2.;
                        height = cuboid.half_extents.y * 2.;
                        style = format!("");
                    } else if let Some(capsule) = shape.as_capsule() {
                        width = capsule.radius * 2.;
                        height = capsule.height() + capsule.radius * 2.;
                        style = format!(
                            "border-radius: 30%;", 
                        );
                    } else {
                        style = format!("width: 2rem; height: 2rem; border-radius: 1rem;");
                    }
                    rsx! {
                        div{
                            style: "position: absolute;transform-origin: center;transform: translate({x}rem, {y * -1.}rem) rotate({rotate}rad);border: solid 1px black;width:{width}rem;height:{height}rem;{style}"
                        }
                    }
                }else {
                    rsx! {}
                }
            } else {
                rsx! {}
            }
        });

        let elements = self.elements.iter().map(|((_ch, rh), element)|{
            let rigid = self.rigid_body_set.get(*rh);
            if let Some(rigid) = rigid{
                let (x, y) = (
                    rigid.position().translation.x,
                    rigid.position().translation.y,
                );
                let rotate = rigid.rotation().angle();
            
                rsx! {
                    div{
                        style: "position: absolute;transform-origin: center;transform: translate({x}rem, {y * -1.}rem) rotate({rotate}rad);",
                        {element}
                    }
                }
            }else {
                rsx!{}
            }
        });
        rsx! {
            for e in shapes{
                {e}
            }
            for e in elements {
                {e}
            }
        }
    }
}

