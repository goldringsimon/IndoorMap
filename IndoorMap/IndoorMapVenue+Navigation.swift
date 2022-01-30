//
//  IndoorMapVenue+Extensions.swift
//  IndoorMap
//
//  Created by Andrew Hart on 13/10/2019.
//  Copyright © 2019 Dent Reality. All rights reserved.
//

import Foundation
import CoreLocation

class IndoorMapGraph: Graph {
    
    let vertices: [Vertex]
    private let edgesOutgoing: [String: [IndoorMapEdge]]
    
    init(from venue: IndoorMapVenue) {
        
        let vertices = venue.openings.map {
            IndoorMapVertex(opening: $0)
        }
        self.vertices = vertices
        
        // For each opening
        self.edgesOutgoing = .init(uniqueKeysWithValues: vertices.map { currentNode -> (String, [IndoorMapEdge]) in
            
            // look at all openings that exist
            let edgesOutgoing = vertices
                .filter {
                    // Not interested in the route from this opening to itself
                    guard $0 != currentNode else { return false }
                    
                    // Filter for openings that are connected to this one, by both touching the same unit
                    // TODO: Do we need the other two permutations?
                    return currentNode.opening.origin?.id == $0.opening.origin?.id || currentNode.opening.destination?.id == $0.opening.destination?.id
                }
                .map { connectedOpening -> IndoorMapEdge in
                    IndoorMapEdge(source: currentNode, target: connectedOpening)
                }
            
            return (currentNode.id, edgesOutgoing)
        })
    }
    
    func edgesOutgoing(from vertex: IndoorMapVertex) -> [IndoorMapEdge] {
        edgesOutgoing[vertex.opening.id] ?? []
    }
}

struct IndoorMapVertex {
    let opening: IndoorMapOpening
    
    var id: String {
        opening.id
    }
}

extension IndoorMapVertex: Hashable {
    static func == (lhs: IndoorMapVertex, rhs: IndoorMapVertex) -> Bool {
        lhs.opening.id == rhs.opening.id
    }
    
    func hash(into hasher: inout Hasher) {
        hasher.combine(opening.id)
    }
}

struct IndoorMapEdge: WeightedEdge {
    let cost: Double
    let target: IndoorMapVertex
    
    internal init(source: IndoorMapVertex, target: IndoorMapVertex) {
        self.cost = source.opening.coordinate.distance(to: target.opening.coordinate)
        self.target = target
    }
}

extension IndoorMapVenue {
	func findRoute(from: IndoorMapUnit, to: IndoorMapUnit) -> [CLLocationCoordinate2D] {
        
        // Get from unit to first opening
        
        /* Find path through openings and units (WIP test code here)
         
        let graph = IndoorMapGraph(from: self)
        
        let aStar = AStar(graph: graph, heuristic: { (vertex1, vertex2) -> Double in
            return vertex1.opening.coordinate.distance(to: vertex2.opening.coordinate)
        })
        
        let path = aStar.path(start: graph.vertices.first!, target: graph.vertices.last!)
         */
        
        // Transform path of openings and units to list of coordinates.
        // Trivial for openings
        // For units, find a path in the unit's coordinate polygon from origin to destination opening
        // Geometry problem, then maybe another path-finding problem?
        // E.g. split into convex polygons, create list of midpoint coordinates, then flatMap that list of lists of coordinates into our return value
        
        return []
	}
}
