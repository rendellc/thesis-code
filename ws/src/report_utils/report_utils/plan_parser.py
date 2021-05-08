import json

from collections import namedtuple
from collections.abc import Sequence

import numpy as np
import pyproj

Waypoint = namedtuple("Waypoint", ["type", "lat", "lon"])


def simple_item_parser(item):
    if item["command"] == 22:
        return Waypoint(
            "start", item["params"][4], item["params"][5]
        )
    elif item["command"] == 16:
        assert len(item["params"]) == 7
        return Waypoint(
            "move", item["params"][4], item["params"][5]
        )
    elif item["command"] == 20:
        return Waypoint(
            "rtl", None, None
        )
    else:
        ignored_commands = [
            530, 206,  # camera stuff
        ]
        if item["command"] not in ignored_commands:
            print(f"command not found ({item['command']})")

    return None


def parse_transect_style_complex_item(data):
    return list(map(simple_item_parser, data["Items"]))


def parse(plan_path):
    with open(plan_path, "r") as file:
        data = json.load(file)

    waypoints = []

    home_data = data["mission"]["plannedHomePosition"]
    home_item = Waypoint("home", home_data[0], home_data[1])

    items = data["mission"]["items"]
    waypoints = []
    for item in items:
        if item["type"] == "SimpleItem":
            item_data = simple_item_parser(item)
            waypoints.append(item_data)
        elif item["type"] == "ComplexItem":
            if item["complexItemType"] == "survey":
                item_datas = parse_transect_style_complex_item(
                    item["TransectStyleComplexItem"])
                waypoints.extend(item_datas)
            else:
                raise RuntimeError("ComplexItem encountered")

    waypoints = list(filter(lambda wp: wp != None, waypoints))
    for i, wp in enumerate(waypoints):
        if wp.type == "rtl":
            waypoints[i] = Waypoint(
                "move", home_item.lat, home_item.lon
            )

    return waypoints, home_item


def versin(theta):
    return 1 - np.cos(theta)


def hav(theta):
    return versin(theta)/2


def hav2(phi1, lam1, phi2, lam2):
    return hav(phi2 - phi1) + np.cos(phi1)*np.cos(phi2)*hav(lam2 - lam1)


def archav(h):
    return 2*np.arcsin(np.sqrt(h))


def latlon_to_cartesian(lat, lon, lat_home, lon_home):
    R_earth = 6371*1000
    C_earth = 2*np.pi*R_earth

    phi, lam = np.deg2rad(lat), np.deg2rad(lon)
    phi0, lam0 = np.deg2rad(lat_home), np.deg2rad(lon_home)

    h_phi = hav2(phi0, lam0, phi, lam0)
    distance_lat = R_earth*archav(h_phi)
    h_lam = hav2(phi0, lam0, phi0, lam)
    distance_lon = R_earth*archav(h_lam)

    dx = distance_lat*np.sign(phi-phi0)
    dy = distance_lon*np.sign(lam-lam0)

    # https://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
    #     # Constants
    #     N0 = 0
    #     E0 = 500
    #     a = R_earth
    #     f = 1/298.257223563
    #     n = f/(2 - f)
    #     A = a/(1 + n)*(1 + n**2/4 + n**4/64)
    #     k0 = 0.9996
    #     alpha = [None]
    #     alpha.append(1/2*n - 2/3*n**2 + 5/16*n**3)
    #     alpha.append(13/48*n**2 - 3/5*n**3)
    #     alpha.append(61/240*n**3)
    #     beta = [None]
    #     beta.append(1/2*n - 2/3*n**2 + 37/96*n**3)
    #     beta.append(1/48*n**2 + 1/15*n**3)
    #     beta.append(17/480*n**3)
    #     delta = [None]
    #     delta.append(2*n - 2/3*n**2 - 2*n**3)
    #     delta.append(7/3*n**2 - 8/5*n**3)
    #     delta.append(56/15*n**3)
    #
    # # Variables
    # phi0 = lat_home
    # lam0 = lon_home
    # phi = lat
    # lam = lon
    # t = np.sinh(np.arctanh(np.sin(phi))) - 2*np.sqrt(n)/(1 + n) * \
    #     np.arctanh(2*np.sqrt(n)/(1 + n)*np.sin(phi))
    # ksi_prime = np.arctan(t/(np.cos(lam - lam0)))
    # etta_prime = np.arctanh(np.sin(lam - lam0)/(np.sqrt(1 + t**2)))
    # sigma = 1 + sum([
    #     2*j*alpha[j]*np.cos(2*j*ksi_prime)*np.cosh(2*j*etta_prime)
    #     for j in range(1, 4)
    # ])
    # tau = sum([
    #     2*j*alpha[j]*np.sin(2*j*ksi_prime)*np.sinh(2*j*etta_prime)
    #     for j in range(1, 4)
    # ])

    # E = E0 + k0*A*(etta_prime + sum([
    #     alpha[j]*np.cos(2*j*ksi_prime)*np.sinh(2*j*etta_prime)
    #     for j in range(1, 4)
    # ]))
    # N = N0 + k0*A*(ksi_prime + sum([
    #     alpha[j]*np.sin(2*j*ksi_prime)*np.cosh(2*j*etta_prime)
    #     for j in range(1, 4)
    # ]))
    # dx = E/1000
    # dy = N/1000

    # d_lat = np.deg2rad(lat - lat_home)
    # d_lon = np.deg2rad(lon - lon_home)

    # # Use ENU convention
    # dx = R_earth*np.sin(d_lat)*np.cos(d_lon)
    # dy = R_earth*np.sin(d_lon)*np.cos(d_lat)
    # x, y = pyproj.transform(wgs84, epsg3035, lon, lat)
    # xh, yh = pyproj.transform(wgs84, epsg3035, lon_home, lat_home)
    # dx, dy = x - xh, y-yh

    return dx, dy


def plan_file_to_cartesian(path_file):
    waypoints, home = parse(path_file)
    xys = np.empty((len(waypoints), 2))
    for i, wp in enumerate(waypoints):
        xys[i] = latlon_to_cartesian(wp.lat, wp.lon, home.lat, home.lon)

    return xys


if __name__ == "__main__":
    xys = plan_file_to_cartesian(
        "/home/cale/thesis-code/ws/launch/survey_dragvoll_manual.plan")
    # print(xys)

    import matplotlib.pyplot as plt
    plt.plot(xys[:, 1], xys[:, 0])
    plt.show()
