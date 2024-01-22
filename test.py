import geojson

def xy_2_line(input, output):
    with open(input) as file:
        points = []
        for line in file:
            data = line.split(" ")
            x = float(data[0])
            y = float(data[1])
            points.append(geojson.Point((x, y), precision = 10))
        gline = geojson.LineString(points)
        feature = geojson.Feature(geometry = gline)
        with open(output, 'w') as of:
            geojson.dump(feature, of)

# xy_2_line("build/frenet_reference_line.txt", "build/frenet_reference_line.geojson")
# xy_2_line("build/output.txt", "build/spline.geojson")
xy_2_line("build/output-11.txt", "build/spline-11.geojson")
# xy_2_line("build/debug-concave.txt", "build/debug-concave.geojson")
# xy_2_line("build/debug-reference-line.txt", "build/debug-reference-line.geojson")
