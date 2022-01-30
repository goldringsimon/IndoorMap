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
        /*
        let graph = IndoorMapGraph(from: self)
        
        let aStar = AStar(graph: graph, heuristic: { (vertex1, vertex2) -> Double in
            return vertex1.opening.coordinate.distance(to: vertex2.opening.coordinate)
        })
        
        let path = aStar.path(start: graph.vertices.first!, target: graph.vertices.last!)
         */
        
        return []
	}
}

public protocol Graph {
    associatedtype Vertex: Hashable
    associatedtype Edge: WeightedEdge where Edge.Vertex == Vertex

    /// Lists all edges going out from a vertex.
    func edgesOutgoing(from vertex: Vertex) -> [Edge]
}

public protocol WeightedEdge {
    associatedtype Vertex

    /// The edge's cost.
    var cost: Double { get }

    /// The target vertex.
    var target: Vertex { get }
}

public final class AStar<G: Graph> {
    /// The graph to search on.
    public let graph: G

    /// The heuristic cost function that estimates the cost between two vertices.
    ///
    /// - Note: The heuristic function needs to always return a value that is lower-than or equal to the actual
    ///         cost for the resulting path of the A* search to be optimal.
    public let heuristic: (G.Vertex, G.Vertex) -> Double

    /// Open list of nodes to expand.
    private var open: HashedHeap<Node<G.Vertex>>

    /// Closed list of vertices already expanded.
    private var closed = Set<G.Vertex>()

    /// Actual vertex cost for vertices we already encountered (refered to as `g` on the literature).
    private var costs = Dictionary<G.Vertex, Double>()

    /// Store the previous node for each expanded node to recreate the path.
    private var parents = Dictionary<G.Vertex, G.Vertex>()

    /// Initializes `AStar` with a graph and a heuristic cost function.
    public init(graph: G, heuristic: @escaping (G.Vertex, G.Vertex) -> Double) {
        self.graph = graph
        self.heuristic = heuristic
        open = HashedHeap(sort: <)
    }

    /// Finds an optimal path between `source` and `target`.
    ///
    /// - Precondition: both `source` and `target` belong to `graph`.
    public func path(start: G.Vertex, target: G.Vertex) -> [G.Vertex] {
        open.insert(Node<G.Vertex>(vertex: start, cost: 0, estimate: heuristic(start, target)))
        while !open.isEmpty {
            guard let node = open.remove() else {
                break
            }
            costs[node.vertex] = node.cost

            if (node.vertex == target) {
                let path = buildPath(start: start, target: target)
                cleanup()
                return path
            }

            if !closed.contains(node.vertex) {
                expand(node: node, target: target)
                closed.insert(node.vertex)
            }
        }

        // No path found
        return []
    }

    private func expand(node: Node<G.Vertex>, target: G.Vertex) {
        let edges = graph.edgesOutgoing(from: node.vertex)
        for edge in edges {
            let g = cost(node.vertex) + edge.cost
            if g < cost(edge.target) {
                open.insert(Node<G.Vertex>(vertex: edge.target, cost: g, estimate: heuristic(edge.target, target)))
                parents[edge.target] = node.vertex
            }
        }
    }

    private func cost(_ vertex: G.Edge.Vertex) -> Double {
        if let c = costs[vertex] {
            return c
        }

        let node = Node(vertex: vertex, cost: Double.greatestFiniteMagnitude, estimate: 0)
        if let index = open.index(of: node) {
            return open[index].cost
        }

        return Double.greatestFiniteMagnitude
    }

    private func buildPath(start: G.Vertex, target: G.Vertex) -> [G.Vertex] {
        var path = Array<G.Vertex>()
        path.append(target)

        var current = target
        while current != start {
            guard let parent = parents[current] else {
                return [] // no path found
            }
            current = parent
            path.append(current)
        }

        return path.reversed()
    }

    private func cleanup() {
        open.removeAll()
        closed.removeAll()
        parents.removeAll()
    }
}

private struct Node<V: Hashable>: Hashable, Comparable {
    /// The graph vertex.
    var vertex: V

    /// The actual cost between the start vertex and this vertex.
    var cost: Double

    /// Estimated (heuristic) cost betweent this vertex and the target vertex.
    var estimate: Double

    public init(vertex: V, cost: Double, estimate: Double) {
        self.vertex = vertex
        self.cost = cost
        self.estimate = estimate
    }

    static func < (lhs: Node<V>, rhs: Node<V>) -> Bool {
        return lhs.cost + lhs.estimate < rhs.cost + rhs.estimate
    }

    static func == (lhs: Node<V>, rhs: Node<V>) -> Bool {
        return lhs.vertex == rhs.vertex
    }

    var hashValue: Int {
        return vertex.hashValue
    }
}

// MARK: - Hashed Heap

/// Heap with an index hash map (dictionary) to speed up lookups by value.
///
/// A heap keeps elements ordered in a binary tree without the use of pointers. A hashed heap does that as well as
/// having amortized constant lookups by value. This is used in the A* and other heuristic search algorithms to achieve
/// optimal performance.
public struct HashedHeap<T: Hashable> {
  /// The array that stores the heap's nodes.
  private(set) var elements = [T]()
  
  /// Hash mapping from elements to indices in the `elements` array.
  private(set) var indices = [T: Int]()
  
  /// Determines whether this is a max-heap (>) or min-heap (<).
  fileprivate var isOrderedBefore: (T, T) -> Bool
  
  /// Creates an empty hashed heap.
  ///
  /// The sort function determines whether this is a min-heap or max-heap. For integers, > makes a max-heap, < makes
  /// a min-heap.
  public init(sort: @escaping (T, T) -> Bool) {
    isOrderedBefore = sort
  }
  
  /// Creates a hashed heap from an array.
  ///
  /// The order of the array does not matter; the elements are inserted into the heap in the order determined by the
  /// sort function.
  ///
  /// - Complexity: O(n)
  public init(array: [T], sort: @escaping (T, T) -> Bool) {
    isOrderedBefore = sort
    build(from: array)
  }
  
  /// Converts an array to a max-heap or min-heap in a bottom-up manner.
  ///
  /// - Complexity: O(n)
  private mutating func build(from array: [T]) {
    elements = array
    for index in elements.indices {
      indices[elements[index]] = index
    }
    
    for i in stride(from: (elements.count/2 - 1), through: 0, by: -1) {
      shiftDown(i, heapSize: elements.count)
    }
  }
  
  /// Whether the heap is empty.
  public var isEmpty: Bool {
    return elements.isEmpty
  }
  
  /// The number of elements in the heap.
  public var count: Int {
    return elements.count
  }

  /// Accesses an element by its index.
  public subscript(index: Int) -> T {
    return elements[index]
  }

  /// Returns the index of the given element.
  ///
  /// This is the operation that a hashed heap optimizes in compassion with a normal heap. In a normal heap this
  /// would take O(n), but for the hashed heap this takes amortized constatn time.
  ///
  /// - Complexity: Amortized constant
  public func index(of element: T) -> Int? {
    return indices[element]
  }
  
  /// Returns the maximum value in the heap (for a max-heap) or the minimum value (for a min-heap).
  ///
  /// - Complexity: O(1)
  public func peek() -> T? {
    return elements.first
  }
  
  /// Adds a new value to the heap.
  ///
  /// This reorders the heap so that the max-heap or min-heap property still holds.
  ///
  /// - Complexity: O(log n)
  public mutating func insert(_ value: T) {
    elements.append(value)
    indices[value] = elements.count - 1
    shiftUp(elements.count - 1)
  }
  
  /// Adds new values to the heap.
  public mutating func insert<S: Sequence>(_ sequence: S) where S.Iterator.Element == T {
    for value in sequence {
      insert(value)
    }
  }
  
  /// Replaces an element in the hash.
  ///
  /// In a max-heap, the new element should be larger than the old one; in a min-heap it should be smaller.
  public mutating func replace(_ value: T, at index: Int) {
    guard index < elements.count else { return }
    
    assert(isOrderedBefore(value, elements[index]))
    set(value, at: index)
    shiftUp(index)
  }
  
  /// Removes the root node from the heap.
  ///
  /// For a max-heap, this is the maximum value; for a min-heap it is the minimum value.
  ///
  /// - Complexity: O(log n)
  @discardableResult
  public mutating func remove() -> T? {
    if elements.isEmpty {
      return nil
    } else if elements.count == 1 {
      return removeLast()
    } else {
      // Use the last node to replace the first one, then fix the heap by
      // shifting this new first node into its proper position.
      let value = elements[0]
      set(removeLast(), at: 0)
      shiftDown()
      return value
    }
  }
  
  /// Removes an arbitrary node from the heap.
  ///
  /// You need to know the node's index, which may actually take O(n) steps to find.
  ///
  /// - Complexity: O(log n).
  public mutating func remove(at index: Int) -> T? {
    guard index < elements.count else { return nil }
    
    let size = elements.count - 1
    if index != size {
      swapAt(index, size)
      shiftDown(index, heapSize: size)
      shiftUp(index)
    }
    return removeLast()
  }

  /// Removes all elements from the heap.
  public mutating func removeAll() {
    elements.removeAll()
    indices.removeAll()
  }
  
  /// Removes the last element from the heap.
  ///
  /// - Complexity: O(1)
  public mutating func removeLast() -> T {
    guard let value = elements.last else {
      preconditionFailure("Trying to remove element from empty heap")
    }
    indices[value] = nil
    return elements.removeLast()
  }
  
  /// Takes a child node and looks at its parents; if a parent is not larger (max-heap) or not smaller (min-heap)
  /// than the child, we exchange them.
  mutating func shiftUp(_ index: Int) {
    var childIndex = index
    let child = elements[childIndex]
    var parentIndex = self.parentIndex(of: childIndex)
    
    while childIndex > 0 && isOrderedBefore(child, elements[parentIndex]) {
      set(elements[parentIndex], at: childIndex)
      childIndex = parentIndex
      parentIndex = self.parentIndex(of: childIndex)
    }
    
    set(child, at: childIndex)
  }
  
  mutating func shiftDown() {
    shiftDown(0, heapSize: elements.count)
  }
  
  /// Looks at a parent node and makes sure it is still larger (max-heap) or smaller (min-heap) than its childeren.
  mutating func shiftDown(_ index: Int, heapSize: Int) {
    var parentIndex = index
    
    while true {
      let leftChildIndex = self.leftChildIndex(of: parentIndex)
      let rightChildIndex = leftChildIndex + 1
      
      // Figure out which comes first if we order them by the sort function:
      // the parent, the left child, or the right child. If the parent comes
      // first, we're done. If not, that element is out-of-place and we make
      // it "float down" the tree until the heap property is restored.
      var first = parentIndex
      if leftChildIndex < heapSize && isOrderedBefore(elements[leftChildIndex], elements[first]) {
        first = leftChildIndex
      }
      if rightChildIndex < heapSize && isOrderedBefore(elements[rightChildIndex], elements[first]) {
        first = rightChildIndex
      }
      if first == parentIndex { return }
      
      swapAt(parentIndex, first)
      parentIndex = first
    }
  }
  
  /// Replaces an element in the heap and updates the indices hash.
  private mutating func set(_ newValue: T, at index: Int) {
    indices[elements[index]] = nil
    elements[index] = newValue
    indices[newValue] = index
  }
  
  /// Swap two elements in the heap and update the indices hash.
  private mutating func swapAt(_ i: Int, _ j: Int) {
    elements.swapAt(i, j)
    indices[elements[i]] = i
    indices[elements[j]] = j
  }
  
  /// Returns the index of the parent of the element at index i.
  ///
  /// - Note: The element at index 0 is the root of the tree and has no parent.
  @inline(__always)
  func parentIndex(of index: Int) -> Int {
    return (index - 1) / 2
  }
  
  /// Returns the index of the left child of the element at index i.
  ///
  /// - Note: this index can be greater than the heap size, in which case there is no left child.
  @inline(__always)
  func leftChildIndex(of index: Int) -> Int {
    return 2*index + 1
  }
  
  /// Returns the index of the right child of the element at index i.
  ///
  /// - Note: this index can be greater than the heap size, in which case there is no right child.
  @inline(__always)
  func rightChildIndex(of index: Int) -> Int {
    return 2*index + 2
  }
}
