"""Public startup selection, dependency expansion, and session RViz generation."""
from pathlib import Path

import pytest
import yaml

from simulation import cli, map_visualization as view, profiles
from simulation.manual import runtime_command


@pytest.mark.parametrize('mode', dict(view.MODES))
def test_public_cli_saved_configuration_to_runtime_and_rviz(mode, tmp_path):
    config_path = tmp_path/'profile.json'
    assert cli.main(['run', 'full', '--no-people', '--map-color', mode, '--map-slice-height', '1.4',
                     '--map-height-min', '-1', '--map-height-max', '5',
                     '--save', str(config_path), '--dry-run']) == 0
    config = profiles.read_config(config_path)
    options = config['options']
    command = profiles.command(config)
    assert command[command.index('--map-color')+1] == mode
    assert options['color_map'] is (mode == 'rgb')
    assert options['people'] is (mode == 'semantic')
    command = runtime_command(dict(device='/dev/input/js0', input_profile='pocket-rc-v2'),
                              3600, program='full', color_map=options['color_map'],
                              person_model=Path('/fixture/model.onnx') if options['people'] else None,
                              map_options={key: options[key] for key in ['map_color', *(s[0] for s in view.SCALES)]})
    assert command[command.index('--map-color')+1] == mode
    if mode == 'rgb':
        assert '--color-map' in command
    if mode == 'semantic':
        assert '--people' in command
    source = profiles.ROOT/'src/uav_bringup/rviz/px4_vio_sim.rviz'
    original = source.read_bytes()
    destination = view.write_rviz(source, tmp_path/'session.rviz', options)
    assert source.read_bytes() == original
    displays = yaml.safe_load(destination.read_text())['Visualization Manager']['Displays']
    selected = next(d for d in displays if d['Name'].startswith('Live Map:'))
    cls, topic, color = view.display_spec(mode)
    assert (selected['Class'], selected['Topic']['Value']) == (cls, topic)
    if color:
        assert selected['Mesh Color'] == color
        assert selected['Min Height'] == -1.
        assert selected['Max Height'] == 5.
    assert bool(view.node_parameters(options)) == (mode in ('age', 'weight', 'semantic', 'esdf', 'traversability'))


@pytest.mark.parametrize('options', [dict(map_color='confidence'), dict(map_height_min=4.),
    dict(map_clearance=0.), dict(map_age_max=float('nan')), dict(map_slice_height=float('inf')),
    dict(map_caution_margin=-1.), dict(map_semantic_radius=True)])
def test_invalid_colors_and_scales_rejected_before_launch(options):
    with pytest.raises(ValueError):
        profiles.validate_config(dict(schema=1, profile='full', options=options))


def test_manual_has_no_map_options():
    with pytest.raises(ValueError):
        profiles.validate_config(dict(schema=1, profile='manual', options=dict(map_color='height')))


def test_mapping_graph_starts_only_selected_derived_view(tmp_path):
    from vio_chain_mapping import launch_mapping
    (tmp_path/'assets').mkdir()
    (tmp_path/'assets/default.sdf').write_text('<sdf/>')
    for mode, _ in view.MODES:
        calls = {}
        options = profiles.validate_config(dict(schema=1, profile='full', options=dict(map_color=mode)))['options']
        launch_mapping(lambda name, cmd, env: calls.update({name: cmd}), {}, tmp_path, 'fixture',
                       graph_only=True, visualize=True, color_map=options['color_map'], map_options=options)
        assert ('map_visualizer' in calls) == bool(view.node_parameters(options))
        assert f'use_color:={str(mode == "rgb").lower()}' in calls['nvblox']
        if 'map_visualizer' in calls:
            assert f'mode:={mode}' in calls['map_visualizer']
        assert 'person_mapper' not in calls  # detection belongs to the later depth stage
