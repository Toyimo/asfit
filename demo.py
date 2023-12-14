import geojson

def xy_2_line():
    with open("build/output.txt") as file:
        points = []
        for line in file:
            data = line.split(" ")
            x = float(data[0])
            y = float(data[1])
            points.append(geojson.Point((x, y), precision = 10))
        gline = geojson.LineString(points)
        feature = geojson.Feature(geometry = gline)
        with open("build/spline.geojson", 'w') as of:
            geojson.dump(feature, of)

xy_2_line()