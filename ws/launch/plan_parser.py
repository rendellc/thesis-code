import json

from collections import namedtuple
from collections.abc import Sequence

import numpy as np

Waypoint = namedtuple("Waypoint", ["type", "lat", "lon"])


def simple_item_parser(item):
    if item["command"] == 22:
        return Waypoint(
            "start", item["params"][4], item["params"][5]
        )
    elif item["command"] == 16:
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


def latlon_to_cartesian(lat, lon, lat_home, lon_home):
    R_earth = 6371*1000

    d_lat = np.deg2rad(lat - lat_home)
    d_lon = np.deg2rad(lon - lon_home)

    # Use ENU convention
    x = R_earth*np.sin(d_lat)
    y = R_earth*np.sin(d_lon)

    return x, y


def plan_file_to_cartesian(path_file):
    waypoints, home = parse("survey_dragvoll.plan")
    xys = np.empty((len(waypoints), 2))
    for i, wp in enumerate(waypoints):
        xys[i] = latlon_to_cartesian(wp.lat, wp.lon, home.lat, home.lon)

    return xys


if __name__ == "__main__":
    xys = plan_file_to_cartesian("survey_dragvoll.plan")
    print(xys)
