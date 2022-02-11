import re
import sys
from heapq import heappop, heappush
from math import sqrt
from PIL import Image, ImageDraw

"""
file: lab1.py
description: This is a program to find to find the optimal path in a map using A* algorithm and BFS
language: python3
author: Aditya Ajit Tirakannavar(at2650@rit.edu)
"""

# dimensions(meters) to measure path distance
Latitude = 7.55
Longitude = 10.29
# terrain color codes
Openland = (248,148,18)
Roughmeadow = (255,192,0)
Easymovementforest = (255,255,255)
Slowrunforest = (2,208,60)
Walkforest = (2,136,40)
Impassablevegetation = (5,73,24)
LakeSwampMarsh = (0,0,255)
Pavedroad = (71,51,3)
Footpath = (0,0,0)
Outofbounds = (205,0,101)


def getPixel(imageName):
    """
    Method which reads the image file and returns the pixel values using Image library functions
    :param imageName: The terrain map image
    :return: pixel: list
    """
    with Image.open(imageName) as img:
        pixel = []
        for h in range(img.height):
            elevation = []
            for w in range(img.width):
                val = tuple(img.getpixel((w, h))[d] for d in range(len(img.getpixel((w, h))) - 1))
                elevation.append(val)
            pixel.append(elevation)
    return pixel

def getPath(fileName):
    """
    Method which reads the path text file and returns the list of x , y (coordinates)values
    :param fileName: path file contatining coordinates
    :return: path: list
    """
    pathCoordinates = []
    with open(fileName) as f:
        for line in f:
            cordinates = line.strip().split(' ')
            pathCoordinates.append((int(cordinates[0]), int(cordinates[1])))
    return pathCoordinates


def getElevation(fileName):
    """
    Method which reads the elevations text file and returns the list of elevation values(float)
    :param fileName: path file contatining elevations of the map
    :return: mapElevations: list
    """
    mapElevations = []
    with open(fileName) as f:
        for line in f:
            temp = []
            # non white space chars are extracted using \s+ regex
            someList = re.split('\s+', line)
            for elevation in someList:
                if elevation != '':
                    val = float(elevation)
                    temp.append(val)
            mapElevations.append(temp)
    return mapElevations


def getTerrains():
    """
    This Method sets the predefined speed/terrain modifier value for each terrain type
    :return: terrain: dict
    """
    terrain = dict()
    terrain[Openland] = 0.2
    terrain[Roughmeadow] = 0.7
    terrain[Easymovementforest] = 0.3
    terrain[Slowrunforest] = 0.5
    terrain[Walkforest] = 0.4
    terrain[Impassablevegetation] = -1
    terrain[LakeSwampMarsh] = -1
    terrain[Pavedroad] = 0.1
    terrain[Footpath] = 0.1
    terrain[Outofbounds] = -1
    return terrain

def getDistance(source, target, elevation):
    x = abs((int(source[0]) - int(target[0])) ** 2)
    y = abs((int(source[1]) - int(target[1])) ** 2)
    z = abs((elevation[source[1]][source[0]] - elevation[target[1]][target[0]]) ** 2)
    hue = sqrt(x + y + z)
    return hue

def BFS(pixel, source, elevation):
    """
    The method to get the optimal coordinate points which will used to draw the actual path on the map
    :param source: path file contatining elevations of the map
    :param elevation: list of elevation value
    :param pixel: list ofpixel values of the image file
    :return: path: list
    """
    visited = set()
    path = []
    queue = []
    curr = source
    queue.append(source)
    while len(queue) > 0:
        if curr in visited:
            curr = queue.pop(0)
        for point in [(1,0),(0,1),(-1,0),(0,-1)]:
            currPoint = (curr[0] + point[0], curr[1] + point[1])
            if currPoint not in visited and currPoint not in queue:
                if getDistance(source, currPoint, elevation) > 5:
                    return path
                queue.append(currPoint)
                path.append(currPoint)
        visited.add(curr)
    return path


def AStar(source, target, mapdetails, elevation):
    """
    The method is find the shortest optimal path from source to target pixel using A-star
    :param source: source cordinate
    :param target: target coordinate
    :param mapdetails: image pixels
    :param elevation: elevation values
    :return: (final_path,distance): tuple of  coordinate values and distance or cost
    """
    path = dict()
    priorityList = []
    visited = set()
    terrains = getTerrains()
    final_path = []
    path[source] = (None, 0, 0, 0)
    dsitance = ((0, 0), -1)
    priorityList.append((getDistance(source, target, elevation), source))
    pathFlag = False
    while len(priorityList) >= 1 and not pathFlag:
        x = int(source[0])
        y = int(source[1])
        totalcost = path[(x, y)][2]
        tempDist = path[(x, y)][3]
        terrainModifier = terrains.get(mapdetails[y][x])
        # avoiding impassable terrains
        if terrainModifier == -1 or terrainModifier is None:
            visited.add(source)
        currtime = (elevation[y][x] * terrainModifier) / 2
        # if the source cordinates are already in the visisted set, pop the cost from the priority list
        if source in visited:
            source = heappop(priorityList)[1]
            continue
        # iterating in 4 directions and finding the next optiimal path by calculating and comparing the distance between corordinates to finally reach the target
        for coordinate in [(1,0),(0,1),(-1,0),(0,-1)]:
            next = coordinate[0] + x, coordinate[1] + y
            if next not in visited:
                hue = getDistance(next, target, elevation)
                nextTerrainMod = terrains.get(mapdetails[next[1]][next[0]])
                pathValue = path.get(next)
                z = (elevation[next[1]][next[0]] * nextTerrainMod) / 2 + currtime
                # straight/sideways/diagonal distance
                newDistance = tempDist + (Latitude if next[0] == x else Longitude if next[1] == y else (sqrt((( Latitude ) )**2 + (( Longitude ) )**2)))
                newDistance += abs(elevation[next[1]][next[0]] - elevation[y][x])
                if pathValue is None or int(pathValue[1]) > (hue + totalcost + 1 + z):
                    path[next] = (source, hue, totalcost + 1 + z, newDistance)
                # target found
                if hue == 0:
                    dsitance = next
                    pathFlag = True
                    break
                # inserting the updated value in priority list
                heappush(priorityList, (hue + totalcost + 1 + z, next))
        visited.add(source)
        # update source once the its added to visited set and loop till target is reached
        source = heappop(priorityList)[1]
    final_path.append(target)
    final_path.append(dsitance)
    while path.get(final_path[len(final_path) - 1]) is not None and path.get(final_path[len(final_path) - 1])[
        0] is not None:
        final_path.append(path[final_path[len(final_path) - 1]][0])
    return final_path, path[dsitance][3]

def drawPath(outputFile, inputImage, final_path, pixel, path, elevation):
    """
    This method draws the path and inserts the distance in the image and saves the output image file
    :param outputFile: result file with the optimal path and distance
    :param inputImage: input terrain image
    :param pixel
    :param path: path from the input path file
    :param elevation: elevation values
    :return: None
    """
    distance = 0
    inputImg = Image.open(inputImage)
    out = inputImg.copy()
    out.save(outputFile)
    with Image.open(outputFile) as img:
        draw = ImageDraw.Draw(img)
        for point in path:
            for _ in BFS(pixel, point, elevation):
                draw.point(_, "cyan")
        for eachpath in final_path:
            distance += float(eachpath[1])
            for _ in eachpath[0]:
                draw.point(_, "magenta")
        draw.text((10, 10), 'Distance: ' + str(distance), 'black')
        img.save(outputFile)
        img.show()

def main():

    imageName = sys.argv[1]
    elevationfile = sys.argv[2]
    pathfile = sys.argv[3]
    outputfile = sys.argv[4]
    elevation = getElevation(elevationfile)
    path = getPath(pathfile)
    pixel = getPixel(imageName)
    final_path = []
    for i in range(len(path) - 1):
        final_path.append(AStar(path[i], path[i+1], pixel, elevation))
    drawPath(outputfile, imageName, final_path, pixel, path, elevation)

# Main conditional Guard
if __name__ == '__main__':
    main()




