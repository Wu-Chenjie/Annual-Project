"""Shared display colors for Gazebo identities and RViz exploration layers."""

def rgb(value):
    return tuple(int(value[i:i+2], 16)/255. for i in (0, 2, 4))


UAV_COLORS = [rgb('38BDF8'), rgb('F472B6'), rgb('34D399')]
UNKNOWN = rgb('263247')
FREE = rgb('718096')
OCCUPIED = rgb('DEE6EF')
PEER_OBSERVED = rgb('A69BD8')
PRIORITY_LOW = rgb('B89A52')
PRIORITY_HIGH = rgb('FFE08A')
GRAPH_EDGE = rgb('A3B3C7')
GRAPH_NODE = rgb('D5DEEA')
TEXT = rgb('E8EDF4')


def priority_color(strength):
    t = min(1., max(0., strength))
    return tuple(a+(b-a)*t for a, b in zip(PRIORITY_LOW, PRIORITY_HIGH))
