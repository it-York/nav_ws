#!/usr/bin/env python3
"""Height projection for a single flat floor, with local ground references.

Preserves PCD XY coordinates. Ground observations mark free space; unobserved
space stays unknown. Local ground estimation compensates vertical distortion
for projection only; it does not repair a distorted SLAM map.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
import yaml


def read_pcd(path):
    with open(path, 'rb') as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('Missing PCD DATA header')
            parts = line.decode('ascii').strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            header[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        fields = header['FIELDS']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if not all(axis in fields and counts[fields.index(axis)] == 1 for axis in ('x', 'y', 'z')):
            raise ValueError('PCD must contain scalar x/y/z fields')
        if header['DATA'] == ['binary']:
            kinds = {'F': 'f', 'I': 'i', 'U': 'u'}
            dtype = np.dtype([(name, '<'+kinds[kind]+size, (count,))
                              for name, kind, size, count in
                              zip(fields, header['TYPE'], header['SIZE'], counts)])
            data = np.fromfile(stream, dtype=dtype, count=int(header['POINTS'][0]))
            points = np.column_stack([data[axis][:, 0] for axis in ('x', 'y', 'z')])
        elif header['DATA'] == ['ascii']:
            data = np.loadtxt(stream, ndmin=2)
            offsets = np.cumsum([0] + counts[:-1])
            points = data[:, [offsets[fields.index(axis)] for axis in ('x', 'y', 'z')]]
        else:
            raise ValueError('Supported PCD encodings: binary and ascii (not binary_compressed)')
    if len(points) != int(header['POINTS'][0]):
        raise ValueError('PCD point count does not match its header')
    points = points[np.isfinite(points).all(axis=1)].astype(np.float64)
    if not len(points):
        raise ValueError('Empty point cloud')
    return points


def interpolate(tree, values, queries, k=8):
    distance, indices = tree.query(queries, k=min(k, len(values)))
    if distance.ndim == 1:
        return values[indices], distance
    weights = 1 / np.maximum(distance, 0.15)**2
    return (values[indices]*weights).sum(axis=1)/weights.sum(axis=1), distance[:, 0]


def local_ground(points, pose_file, sensor_height=None):
    poses = np.genfromtxt(pose_file, delimiter=',', names=True, ndmin=1)
    trajectory = np.column_stack([poses['map_'+axis] for axis in ('x', 'y', 'z')])
    if not np.isfinite(trajectory).all() or len(trajectory) < 3:
        raise ValueError('At least three valid optimized poses are required')
    tree = cKDTree(trajectory[:, :2])
    distance, nearest = tree.query(points[:, :2])
    relative = points[:, 2]-trajectory[nearest, 2]
    if sensor_height is None:
        sample = relative[(distance < 1.2) & (relative > -1.0) & (relative < -0.25)]
        if len(sample) < 100:
            raise ValueError('Cannot estimate ground; specify --sensor-height or --ground-z')
        bins = np.arange(-1, -0.249, 0.02)
        counts, edges = np.histogram(sample, bins=bins)
        peak = np.argmax(counts)
        sensor_height = -float(np.median(sample[(sample >= edges[peak]-.03) &
                                              (sample <= edges[peak+1]+.03)]))
    # A global trend lets the ground extrapolate gently beyond the driven path;
    # local residuals then follow the observed floor instead of enforcing a plane.
    design = np.column_stack([trajectory[:, :2], np.ones(len(trajectory))])
    plane = np.linalg.lstsq(design, trajectory[:, 2]-sensor_height, rcond=None)[0]
    path_residual = trajectory[:, 2]-sensor_height-design@plane
    trend = points[:, :2]@plane[:2]+plane[2]
    residual, _ = interpolate(tree, path_residual, points[:, :2])
    prior = trend+residual
    seed = (np.abs(points[:, 2]-prior) <= .18) & (distance <= 6.0)
    if seed.sum() < 100:
        raise ValueError('Not enough ground observations near trajectory')
    # Median in 0.75 m cells suppresses individual low returns and small objects.
    tile = np.floor(points[seed, :2]/.75).astype(np.int64)
    cells, inverse, counts = np.unique(tile, axis=0, return_inverse=True, return_counts=True)
    samples = points[seed, 2]-trend[seed]
    order = np.argsort(inverse)
    starts = np.r_[0, np.cumsum(counts)]
    medians = np.array([np.median(samples[order[starts[i]:starts[i+1]]]) for i in range(len(cells))])
    keep = counts >= 3
    centres = (cells[keep]+.5)*.75
    if len(centres) < 4:
        raise ValueError('Insufficient ground tiles')
    local, support_distance = interpolate(cKDTree(centres), medians[keep], points[:, :2], k=6)
    ground = trend+local
    # Distant isolated returns have no measured floor reference: leave unknown.
    supported = support_distance <= 3.0
    details = {'sensor_height_estimate_m': sensor_height, 'ground_tiles': len(centres),
               'ground_z_percentiles_m': np.percentile(ground[supported], [0, 5, 50, 95, 100]).tolist(),
               'trajectory_z_range_m': [float(trajectory[:, 2].min()), float(trajectory[:, 2].max())],
               'global_ground_trend_abc': plane.tolist(),
               'ground_method': 'trajectory_prior_and_local_observed_floor',
               'supported_ground_fraction': float(supported.mean())}
    return ground, supported, details


def disk(radius, resolution):
    n = int(np.ceil(radius/resolution))
    y, x = np.mgrid[-n:n+1, -n:n+1]
    return x*x+y*y <= (radius/resolution)**2+1e-8


def project(floor, obstacles, resolution, occupied_radius, free_radius):
    all_xy = np.vstack([floor[:, :2], obstacles[:, :2]])
    origin = np.floor((all_xy.min(axis=0)-1)/resolution)*resolution
    size = np.ceil((all_xy.max(axis=0)+1-origin)/resolution).astype(int)+1
    if size.prod() > 50_000_000:
        raise ValueError('Grid exceeds 50 million cells; check input or increase resolution')
    def raster(points):
        grid = np.zeros((size[1], size[0]), dtype=bool)
        index = np.floor((points[:, :2]-origin)/resolution).astype(int)
        grid[index[:, 1], index[:, 0]] = True
        return grid
    free = ndimage.binary_dilation(raster(floor), structure=disk(free_radius, resolution))
    occupied = ndimage.binary_dilation(raster(obstacles), structure=disk(occupied_radius, resolution))
    # Occupied always wins. No flood fill of unobserved rooms or obstacle erosion.
    grid = np.full(free.shape, 205, dtype=np.uint8)
    grid[free] = 254
    grid[occupied] = 0
    return np.flipud(grid), origin


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='Output prefix without extension')
    parser.add_argument('--poses', type=Path, help='Optimized keyframe poses.csv')
    parser.add_argument('--ground-z', type=float, help='Use a constant ground Z instead of local ground')
    parser.add_argument('--sensor-height', type=float, help='Sensor-to-floor distance for local ground prior')
    parser.add_argument('--min-height', type=float, default=.15)
    parser.add_argument('--max-height', type=float, default=1.50)
    parser.add_argument('--resolution', type=float, default=.05)
    parser.add_argument('--occupied-radius', type=float, default=.10, help='Point footprint radius, not robot inflation')
    parser.add_argument('--free-radius', type=float, default=.18, help='Observed floor sample footprint radius')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    if not (0 < args.min_height < args.max_height and args.resolution > 0 and
            args.occupied_radius >= 0 and args.free_radius >= 0):
        parser.error('Invalid height, resolution or footprint parameters')
    if args.ground_z is None and not args.poses:
        parser.error('Specify --poses for local ground or --ground-z for a constant plane')
    paths = {ext: Path(str(args.output)+ext) for ext in ('.pgm', '.yaml', '.png', '.report.json')}
    if not args.overwrite and any(p.exists() for p in paths.values()):
        parser.error('Output already exists; use a new prefix or --overwrite')
    points = read_pcd(args.pcd)
    if args.ground_z is not None:
        ground = np.full(len(points), args.ground_z)
        supported = np.ones(len(points), dtype=bool)
        report = {'ground_method': 'constant', 'ground_z_m': args.ground_z}
    else:
        ground, supported, report = local_ground(points, args.poses, args.sensor_height)
    height = points[:, 2]-ground
    floor = points[supported & (np.abs(height) <= .10)]
    obstacles = points[supported & (height >= args.min_height) & (height <= args.max_height)]
    if len(floor) < 100 or len(obstacles) < 10:
        raise ValueError('Too few floor or obstacle points; check height limits')
    # Remove isolated 3D returns without deleting whole small 2D obstacles.
    neighbours, _ = cKDTree(obstacles).query(obstacles, k=3)
    raw_count = len(obstacles)
    obstacles = obstacles[neighbours[:, -1] <= .40]
    if not len(obstacles):
        raise ValueError('No supported obstacle points after outlier filtering')
    grid, origin = project(floor, obstacles, args.resolution, args.occupied_radius, args.free_radius)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(grid).save(paths['.pgm'])
    Image.fromarray(grid).save(paths['.png'])
    metadata = {'image': paths['.pgm'].name, 'mode': 'trinary', 'resolution': args.resolution,
                'origin': [float(origin[0]), float(origin[1]), 0.0], 'negate': 0,
                'occupied_thresh': .65, 'free_thresh': .196}
    paths['.yaml'].write_text(yaml.safe_dump(metadata, sort_keys=False))
    report.update({'source_pcd': str(args.pcd.resolve()),
                   'poses_csv': str(args.poses.resolve()) if args.poses else None,
                   'height_range_above_ground_m': [args.min_height, args.max_height],
                   'resolution_m': args.resolution, 'occupied_radius_m': args.occupied_radius,
                   'free_radius_m': args.free_radius, 'input_points': len(points),
                   'floor_points': len(floor), 'obstacle_points': len(obstacles),
                   'isolated_obstacle_points_removed': raw_count-len(obstacles),
                   'width': grid.shape[1], 'height': grid.shape[0],
                   'origin': metadata['origin'],
                   'cells': {'occupied': int((grid == 0).sum()), 'free': int((grid == 254).sum()),
                             'unknown': int((grid == 205).sum())},
                   'note': 'Local height projection preserves XY; it does not repair SLAM distortion.'})
    paths['.report.json'].write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    print('Map YAML:', paths['.yaml'])


if __name__ == '__main__':
    main()
