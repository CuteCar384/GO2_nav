from go2_navigation.height_map_viz import HeightMapView, convert_height_map_to_points


def test_convert_skips_sentinel_cells():
    hm = HeightMapView(
        frame_id="odom",
        stamp_sec=1.0,
        resolution=0.1,
        width=2,
        height=2,
        origin=(0.0, 0.0),
        data=[0.5, 1e9, 0.2, 0.3],
    )

    points = convert_height_map_to_points(hm)

    assert len(points) == 3
    assert (0.0, 0.0, 0.5) in points
    assert (0.0, 0.1, 0.2) in points
    assert (0.1, 0.1, 0.3) in points


def test_convert_uses_x_major_indexing_and_origin():
    hm = HeightMapView(
        frame_id="odom",
        stamp_sec=2.0,
        resolution=0.06,
        width=3,
        height=2,
        origin=(-1.0, 2.0),
        data=[1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
    )

    points = convert_height_map_to_points(hm)

    assert points == [
        (-1.0, 2.0, 1.0),
        (-0.94, 2.0, 2.0),
        (-0.88, 2.0, 3.0),
        (-1.0, 2.06, 4.0),
        (-0.94, 2.06, 5.0),
        (-0.88, 2.06, 6.0),
    ]
