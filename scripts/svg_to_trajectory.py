import os
import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
import argparse


def load_path_from_svg(file_path, num_points=1000):
    
    first_path, _ = svg2paths(file_path)[0]  # Get the first path object from the SVG file
    complex_points = [first_path.point(t) for t in np.linspace(0, 1, num_points)]
    xy_points = np.array([[pt.real, pt.imag] for pt in complex_points])
    return xy_points


def scale_path(points, target_width):

    original_width = np.ptp(points[:, 0])  # Width is the range of x-coordinates
    scaling_factor = target_width / original_width if original_width > 0 else 1.0
    scaled_points = (points - np.min(points, axis=0)) * scaling_factor
    return scaled_points


def resample_path(points, avg_distance):

    if len(points) < 2:
        return points  # No resampling needed for single point

    resampled_points = [points[0]]  # Start with the first point
    last_point = points[0]

    for point in points[1:]:
        distance = np.linalg.norm(point - last_point)
        if distance >= avg_distance:
            resampled_points.append(point)
            last_point = point

    return np.array(resampled_points)


def compute_orientations(points, threshold=1e-6):
    
    angles = []
    for i in range(1, len(points)):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        if np.linalg.norm([dx, dy]) < threshold:
            angle = angles[-1]  # Keep the same angle if movement is small
        else:
            angle = np.arctan2(dy, dx)
        angles.append(angle)

    angles.insert(0, angles[0])  # First point's angle is the same as the first segment
    xy_yaw_path = np.array([(x, y, angle) for (x, y), angle in zip(points, angles)])
    return xy_yaw_path


def visualize_path(xy_yaw_path, target_width, title="SVG Path Visualization"):
    
    plt.figure(figsize=(10, 10)) # Make plot a bit larger
    plasma_cmap = plt.get_cmap('plasma')
    colors = plasma_cmap(np.linspace(0,1,len(xy_yaw_path)))
    plt.scatter([p[0] for p in xy_yaw_path], [p[1] for p in xy_yaw_path], 
                c=colors, label='Generated Path')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title(title)
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)

    arrow_length = target_width * 0.04 # Make arrow length relative to scale
    head_width = arrow_length * 0.25
    head_length = arrow_length * 0.35

    for i in range(0, len(xy_yaw_path)):
        x, y, angle = xy_yaw_path[i]
        plt.arrow(x, y,
                    arrow_length * np.cos(angle), arrow_length * np.sin(angle),
                    head_width=head_width, head_length=head_length, fc=colors[i], ec=colors[i], alpha=0.7)
    plt.legend()
    plt.show()


def convert_to_ltr(xy_yaw_path, output_directory, input_svg):
    
    output_file = os.path.join(output_directory, os.path.splitext(os.path.basename(input_svg))[0] + ".ltr")
    with open(output_file, 'w') as ltr_file:
        ltr_file.write("#############################\n")
        ltr_file.write(f"frame_id : map\n")  # Assuming 'map' as the frame_id
        for x, y, angle in xy_yaw_path:
            ltr_file.write(f"{x},{y},0.0,0.0,0.0,{np.sin(angle / 2)},{np.cos(angle / 2)}\n")
          
          
def parse_arguments():
    
    parser = argparse.ArgumentParser(description="Convert SVG path to trajectory with optional visualization.")
    parser.add_argument("input_svg", type=str, help="Path to the input SVG file.")
    parser.add_argument("--output", "-o", type=str, default="../trajectories", help="Directory to save the output LTR file.")
    parser.add_argument("--width", "-w", type=float, default=10.0, help="Target width in meters for scaling the path.")
    parser.add_argument("--spacing", "-s", type=float, default=0.1, help="Desired distance between waypoints in meters.")
    parser.add_argument("--preview", "-p", action="store_true", help="Show a plot preview of the path.")

    return parser.parse_args()
            

def main():

    args = parse_arguments()
    
    svg_path = load_path_from_svg(args.input_svg)
    scaled_path = scale_path(svg_path, args.width)
    sampled_path = resample_path(scaled_path, args.spacing)
    xy_yaw_path = compute_orientations(sampled_path)

    if args.preview:
        visualize_path(xy_yaw_path, args.width, title=f"SVG Path from '{os.path.basename(args.input_svg)}'")

    convert_to_ltr(xy_yaw_path, args.output, args.input_svg)


if __name__ == "__main__":
    main()
