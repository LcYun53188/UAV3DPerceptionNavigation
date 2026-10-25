"""Attach actual depth to the VIO aircraft and start the owned mapping graph."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import math
import xml.etree.ElementTree as ET
from sim_validation import ROOT, read, file_hash


def attach_depth(directory, profile, frames, *, independent_rgbd=False):
    path=directory/profile['model']/'model.sdf'
    tree=ET.parse(path)
    model=tree.getroot().find('model')
    rig=model.find("link[@name='vio_rig_link']")
    sensor_parent=rig
    if independent_rgbd:
        # Gazebo pairs CameraSensor subclasses on the same parent link and
        # overwrites P[3] with a stereo baseline. Keep this RGB-D camera out of
        # the VIO stereo group, with its original pose and unchanged VIO rig.
        sensor_parent=ET.SubElement(model,'link',name='chain_rgbd_link')
        ET.SubElement(sensor_parent,'pose',relative_to='vio_rig_link').text='0 0 0 0 0 0'
        inertial=ET.SubElement(sensor_parent,'inertial')
        ET.SubElement(inertial,'mass').text='0.000001'
        inertia=ET.SubElement(inertial,'inertia')
        for axis in ('ixx','iyy','izz'):ET.SubElement(inertia,axis).text='0.000000001'
        joint=ET.SubElement(model,'joint',name='chain_rgbd_joint',type='fixed')
        ET.SubElement(joint,'parent').text='vio_rig_link'
        ET.SubElement(joint,'child').text='chain_rgbd_link'
    frame='oakd_camera_optical_frame'
    sensor=ET.SubElement(sensor_parent,'sensor',name='chain_rgbd',type='rgbd_camera')
    for key,value in [('pose',f'0 0 0 0 {math.radians(5)} 0'),('always_on','true'),
                      ('update_rate','10'),('topic','/px4_reference/rgbd'),('gz_frame_id',frame)]:
        ET.SubElement(sensor,key).text=value
    camera=ET.SubElement(sensor,'camera');ET.SubElement(camera,'horizontal_fov').text=str(profile['horizontal_fov_rad'])
    image=ET.SubElement(camera,'image')
    for key,value in [('width','320'),('height','240'),('format','R8G8B8')]:ET.SubElement(image,key).text=value
    clip=ET.SubElement(camera,'clip')
    for key in ('near','far'):ET.SubElement(clip,key).text=str(profile[key+'_m'])
    ET.SubElement(camera,'camera_info_topic').text='/px4_reference/rgbd/camera_info'
    ET.SubElement(camera,'optical_frame_id').text=frame
    ET.indent(tree);tree.write(path,encoding='utf-8',xml_declaration=True)
    frames[frame]=dict(position=profile['rig_position_flu_m'],rpy=[-math.pi/2-math.radians(5),0.,-math.pi/2])
    (directory/'frames.json').write_text(json.dumps(frames,indent=2)+'\n')
    return dict(width=320,height=240,horizontal_fov_rad=profile['horizontal_fov_rad'],near_m=profile['near_m'],far_m=profile['far_m'],image_rate_hz=10,
        independent_rgbd_link=independent_rgbd,auxiliary_link_mass_kg=1e-6 if independent_rgbd else 0.,
        scope='same-aircraft ideal depth; not hardware OAK-D depth')


def launch_mapping(launch, env, out, run_id, *, graph_only=False, depth_only=False, binding_only=False, visualize=False, person_model=None, color_map=False, map_options=None, person_rotations=False):
    if sum((graph_only,depth_only,binding_only))>1:raise ValueError('Conflicting mapping startup stages')
    transport=dict(env,FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'src/uav_bringup/config/fastdds_map_service.xml'),
                   FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'src/uav_bringup/config/fastdds_map_service.xml'))
    config=read(ROOT/('simulation/safe_regions/'+env.get('UAV_FLIGHT_REGION_PROFILE','vio_chain')+'.json'))
    def start(name, command):
        setup_shell='source "$1/install_uav/setup.bash"; source "$1/.deps/px4-msgs-install/setup.bash"; source "$1/.deps/mission-install/local_setup.bash"; '
        extra_unsets=[]
        if name in ('person_mapper', 'map_visualizer'):
            # This process has no mission/PX4 dependency. Isolate its new messages
            # and type-support libraries from the older mission install entirely.
            setup_shell='source "$1/install_uav/setup.bash"; '
            extra_unsets=['-u','LD_LIBRARY_PATH']
        setup_shell += 'shift; exec "$@"'
        return launch(name,['env','-u','AMENT_PREFIX_PATH','-u','COLCON_PREFIX_PATH','-u','CMAKE_PREFIX_PATH','-u','PYTHONPATH',*extra_unsets,
            'bash','-e','-c',setup_shell,
            'mapping',str(ROOT),*map(str,command)],transport)
    if binding_only:
        start('planning_sources',['ros2','run','uav_nav_sim','planning_sources','--ros-args','-p','use_sim_time:=true',
            '-p',f'localization_session:={run_id}','-p','alignment_id:=depth-map-is-ekf-odom-v1','-p','settle_seconds:=5.0',
            '-p',f'preflight_admission_path:={out/"map-admission.json"}'])
        return
    if not graph_only:start('depth_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
        '/px4_reference/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/px4_reference/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '--ros-args','-r','/px4_reference/rgbd/depth_image:=/px4_depth/image',
        '-r','/px4_reference/rgbd/camera_info:=/px4_depth/camera_info'])
    if (person_model is not None or color_map) and not graph_only:
        if graph_only or binding_only:
            raise ValueError('Start RGB together with the depth input stage')
        # One RGB bridge is shared by optional detection and color fusion.
        start('person_rgb_bridge',['ros2','run','ros_gz_image','image_bridge','/px4_reference/rgbd/image',
            '--ros-args','-p','use_sim_time:=true','-r','/px4_reference/rgbd/image:=/px4_rgb/image'])
    if person_model is not None:
        if graph_only or binding_only:
            raise ValueError('Start person perception together with the depth input stage')
        start('person_mapper',['ros2','run','uav_target_perception','person_mapper','--ros-args',
            '-p','use_sim_time:=true','-p',f'model_path:={person_model}',
            '-p','rgb_topic:=/px4_rgb/image','-p','depth_topic:=/px4_depth/image',
            '-p','camera_info_topic:=/px4_depth/camera_info',
            '-p','context_source:=planning_context','-p','context_topic:=/planning/context',
            '-p',f'rotation_augmentation:={str(person_rotations).lower()}',
            '-p',f'snapshot_path:={out/"person-observations.json"}'])
    if depth_only:return
    # Mesh generation is subscriber driven. Keep headless acceptance unchanged;
    # UI sessions stream actual reconstructed geometry at a bounded rate.
    mesh_rate='0.5' if visualize else '0.0'
    start('nvblox',['ros2','run','nvblox_ros','nvblox_node','--ros-args',
        '--params-file',ROOT/'src/uav_bringup/config/uav_ego_nvblox.yaml','-p','use_sim_time:=true','-p',f'use_color:={str(color_map).lower()}',
        '-p','voxel_size:=0.2','-p','integrate_depth_rate_hz:=5.0',
        *(['-p','integrate_color_rate_hz:=5.0'] if color_map else []),
        '-p','update_mesh_rate_hz:='+mesh_rate,'-p','publish_layer_rate_hz:='+mesh_rate,'-p','publish_debug_vis_rate_hz:=0.0',
        '-p',f'static_mapper.projective_integrator_max_integration_distance_m:={config.get("depth_integration_distance_m",6.0)}',
        '-r','camera_0/depth/image:=/uav/mapping/depth','-r','camera_0/depth/camera_info:=/uav/mapping/camera_info',
        *(['-r','camera_0/color/image:=/uav/mapping/color',
           '-r','camera_0/color/camera_info:=/uav/mapping/color_camera_info'] if color_map else [])])
    start('map_session',['ros2','run','uav_nav_sim','map_session','--ros-args','-p','use_sim_time:=true',
        '-p','resolution:=0.2',
        '-p','operation_guard:=px4_ground',
        '-p','localization_source:=px4_vio_local_enu',
        '-p',f'localization_session:={run_id}','-p','alignment_id:=depth-map-is-ekf-odom-v1',
        '-p',f'preflight_admission_path:={out/"map-admission.json"}',
        '-p',f'scene_id:={file_hash(out/"assets/default.sdf")}',
        '-p','aabb_min:='+json.dumps(list(map(float,config.get('map_aabb_min',[-5.0,-5.0,0.0])))),
        '-p','aabb_size:='+json.dumps(list(map(float,config.get('map_aabb_size',[10.0,10.0,4.0])))),
        '-r','/rgbd_camera/depth_image:=/px4_depth/image','-r','/rgbd_camera/camera_info:=/px4_depth/camera_info',
        *(['-r','/rgbd_camera/image:=/px4_rgb/image'] if color_map else [])])
    if visualize and map_options:
        from simulation.map_visualization import node_parameters
        parameters=node_parameters(map_options)
        if parameters:
            start('map_visualizer',['ros2','run','uav_nav_sim','map_visualizer','--ros-args',
                                    '-p','use_sim_time:=true',*parameters])
    limits=config['ego_limits']
    if not graph_only:launch_mapping(launch,env,out,run_id,binding_only=True,visualize=visualize)
    common=['--ros-args','-p','use_sim_time:=true','-p','body_radius:=0.8',
            '-p',f'max_velocity:={limits[0]}','-p',f'max_acceleration:={limits[1]}','-p',f'max_jerk:={limits[2]}']
    start('planning_context',['ros2','run','uav_nav_sim','planning_context',*common,
        '-r','/planning/source/odometry:=/uav/px4/localized_odometry'])
    command=[ROOT/'install_uav/uav_ego_adapter/lib/uav_ego_adapter/ego_nvblox_planner',*common,
        '-p','coordinate_frame:=map','-p','managed_goals:=true',
        '-p',f'horizontal_braking_margin:={config["braking_margin_m"]}']
    for source,target in {'/uav/map/snapshot':'/planning/ego/map','/uav/localization/odometry':'/planning/ego/odometry',
                         '/uav/goal':'/planning/ego/goal','/uav/trajectory':'/planning/ego/trajectory','/uav/planned_path':'/planning/ego/path'}.items():
        command += ['-r',source+':='+target]
    start('ego_planner',command)


def native_map_file(node, operation, path, timeout=15.):
    """Owned ground-only map lifecycle RPC; caller records actual receipts."""
    import time
    import rclpy
    from nvblox_msgs.srv import FilePath
    client=node.create_client(FilePath,'/nvblox_node/'+operation+'_map')
    until=time.monotonic()+timeout
    try:
        while not client.service_is_ready():
            if time.monotonic()>=until:raise RuntimeError('Map service unavailable: '+operation)
            rclpy.spin_once(node,timeout_sec=.02)
        future=client.call_async(FilePath.Request(file_path=str(path)))
        while not future.done():
            if time.monotonic()>=until:raise RuntimeError('Map RPC timeout: '+operation)
            rclpy.spin_once(node,timeout_sec=.02)
        if not future.result().success:raise RuntimeError('Native map RPC failed: '+operation)
    finally:node.destroy_client(client)


def export_grounded_color_map(node, path, timeout=75.):
    """Export only through MapSession's fresh landed/disarmed guard."""
    import time
    import rclpy
    from nvblox_msgs.srv import FilePath
    path=_Path(path).expanduser().resolve()
    if path.suffix != '.ply' or path.exists():
        raise ValueError('Choose a new .ply export path')
    client=node.create_client(FilePath,'/uav/map/export_mesh')
    until=time.monotonic()+timeout
    try:
        while not client.service_is_ready():
            if time.monotonic()>=until:raise RuntimeError('Guarded color export service unavailable')
            rclpy.spin_once(node,timeout_sec=.02)
        future=client.call_async(FilePath.Request(file_path=str(path)))
        while not future.done():
            if time.monotonic()>=until:raise RuntimeError('Guarded color export timed out; inspect file before retrying')
            rclpy.spin_once(node,timeout_sec=.02)
        if not future.result().success or not path.is_file():
            raise RuntimeError('Guarded color export rejected; inspect MapSession ground/map diagnostics')
        return dict(path=str(path),bytes=path.stat().st_size,sha256=file_hash(path),
                    service='/uav/map/export_mesh',guard='fresh_unique_px4_landed_disarmed')
    finally:node.destroy_client(client)
