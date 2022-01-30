# Notes for findRoute(from:to:) implementation

Task: find route from one room to another, expressed in terms of an array of coordinates.

We split the task into two steps:
1) Find route from one unit to another, expressed in terms of a path-of-units (not coordinates), as permitted by openings
2) Transform the path-of-units to path-of-coordinates, by finding a coordinate-path from entrance to exit opening for each unit

## Task 1: Unit-routing

A shortest-path finding algorithm like Dijkstra's or A-star seems appropriate. Units have a list of coordinates for their perimeters, and openings have a single coordinate. Dealing with a single coordinate sounds simpler, and indeed we are effectively building a route between openings, so using the openings' coordinates seems correct. We'll use 'distance-to-opening's coordinate' as our heuristic, and use the A-star algorithm since we have a heuristic available.

Thinking about function signature:

```
func findRoute(from: IndoorMapUnit, to: IndoorMapUnit) -> [IndoorMapWaypoint]
```

where

```
enum IndoorMapWaypoint {
    case unit(_ unit: IndoorMapUnit)
    case opening(_ opening: IndoorMapOpening)
}
```

The first and last elements of the return value should be `IndoorMapWaypoint.Unit` cases. The return value should be an interleaved list of units and openings. 

## Task 2: Routing from entry to exit opening within each unit in our unit-route
 
Researched: Polygon triangulation, visibility graph, visibility graph path planning. Visibility graph might be overkill... what we really need is is a valid path remaining within a polygon, even if that polygon is convex.
https://graphics.stanford.edu/courses/cs268-09-winter/notes/handout7.pdf
This handout describes shortest path problems. We may not even be concerned with shortest path _within each unit_, as it would seem graphically unsuitable on a map to potentially hug the edges in that manner.
Perhaps we:
1) Triangulate the polygon
2) Build a list of the centres of those triangles ('room-points')
3) Treat these room-points as an undirected graph, and once more use a path search algorithm such as A-star

I think the missing piece is the path from the entrance opening to the first room-point/from the last room-point to the exit opening. How do we know which triangles / room-points are the right starting and ending nodes? (between steps 1 and 2) Maybe it is as simple as distance to opening.

Perhaps we don't want polygon triangulation at all, but instead just to partition each unit (potentially a convex polygon) into convex polygons? Many rooms are rectilinear - we wouldn't want or need to split those into triangles, as a path of the centre of those triangles would be quite a strange way to traverse those rooms!
Investigated: https://doc.cgal.org/latest/Partition_2/index.html#secpartition_2_monotone
In that link, the right hand side of of Figure 19.1 seems like what we want.
