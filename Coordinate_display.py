import folium
import csv
import webbrowser

m = folium.Map(location=[57, 25], zoom_start=10)

with open("datalog422230010.csv") as f:
    reader = csv.reader(f)
    header = next(reader)
    rows = []
    for row in reader:
        rows.append(row)
locations = []
line = 1
for y in rows:
    line += 1
    if line > 695 and y[1] != 0 and y[2] != 0 and [y[1], y[2]] not in locations:
        print([y[1], y[2]], line)
        locations.append([y[1], y[2]])
        folium.Marker(location=[y[1], y[2]]).add_to(m)

m.save("map.html")
webbrowser.open("map.html")
