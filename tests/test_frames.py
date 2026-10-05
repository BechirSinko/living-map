import math

import pytest

from livingmap.frames import (gps_to_local, local_to_enu, local_to_gps,
                              meters_per_degree)

LAT0, LON0 = 36.8065, 10.1815  # Tunis (example anchor)


def test_heading_north_x_goes_north():
    e, n = local_to_enu(10, 0, 0)
    assert (e, n) == pytest.approx((0, 10), abs=1e-9)


def test_heading_north_y_goes_west():
    e, n = local_to_enu(0, 10, 0)
    assert (e, n) == pytest.approx((-10, 0), abs=1e-9)


def test_heading_east_x_goes_east_y_goes_north():
    assert local_to_enu(10, 0, 90) == pytest.approx((10, 0), abs=1e-9)
    assert local_to_enu(0, 10, 90) == pytest.approx((0, 10), abs=1e-9)


def test_distance_preserved():
    e, n = local_to_enu(30, 40, 37)
    assert math.hypot(e, n) == pytest.approx(50)


def test_origin_maps_to_anchor():
    assert local_to_gps(0, 0, LAT0, LON0, 123) == (LAT0, LON0)


def test_100m_north_is_about_0_0009_deg():
    lat, lon = local_to_gps(100, 0, LAT0, LON0, 0)
    assert lon == pytest.approx(LON0, abs=1e-12)
    assert lat - LAT0 == pytest.approx(100 / meters_per_degree(LAT0)[0])
    assert 0.00089 < lat - LAT0 < 0.00091


@pytest.mark.parametrize("heading", [0, 45, 90, 181, 270, 359])
def test_round_trip(heading):
    x, y = 123.4, -56.7
    lat, lon = local_to_gps(x, y, LAT0, LON0, heading)
    x2, y2 = gps_to_local(lat, lon, LAT0, LON0, heading)
    assert (x2, y2) == pytest.approx((x, y), abs=1e-6)
