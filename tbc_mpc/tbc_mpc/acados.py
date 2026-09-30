#!/usr/bin/env python3

'''
A wrapper around acados to make it more straightforward during labs
'''


def get_solvers(pref = None):
    import os
    import json
    libs_file = os.environ['ACADOS_SOURCE_DIR'] + '/lib/link_libs.json'
    with open(libs_file) as f:
        link_libs = json.load(f)

    solvers = {'partial': ['hpipm', 'qpdunes','osqp','clarabel'],
               'full': ['qpoases', 'hpipm','daqp']}

    avail = []

    for base, libs in solvers.items():
        for lib in libs:
            if link_libs.get(lib, True):
                avail.append(f'{base}_condensing_{lib}'.upper())

    if pref in avail:
        return [pref]

    return avail
