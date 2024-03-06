#include "student_code.h"
#include "CGL/vector3D.h"
#include "halfEdgeMesh.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    if (points.size() == 1) {
      return points;
    }
    vector<Vector2D> newPoints(points.size() - 1);
    for (int i = 0; i < points.size() - 1; i++) {
      newPoints[i] = (1 - t) * points[i] + t * points[i + 1];
    }
    return newPoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    vector<Vector3D> newPoints(points.size() - 1);
    for (int i = 0; i < points.size() - 1; i++) {
      newPoints[i] = (1 - t) * points[i] + t * points[i + 1];
    }
    return newPoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    if (points.size() == 1) {
      return points[0];
    }
    vector<Vector3D> newPoints = evaluateStep(points, t);
    return evaluate1D(newPoints, t);
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> bezierCurvePoints(controlPoints.size());
    for (int i = 0; i < controlPoints.size(); i++) {
      bezierCurvePoints[i] = evaluate1D(controlPoints[i], u);
    }
    return evaluate1D(bezierCurvePoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    HalfedgeCIter h = halfedge(); // current out-going half-edge of vertex
    
    Vector3D sumNormal(0, 0, 0); // sum of weighted normals

    do {
      HalfedgeCIter h_twin = h->twin(); // twin of half-edge

      Vector3D v0 = position;
      Vector3D v1 = h->next()->vertex()->position;
      Vector3D v2 = h->next()->next()->vertex()->position;
    
      Vector3D normalVector = cross(v1 - v0, v2 - v0); // normal vector of triangle
      double area = 0.5 * normalVector.norm(); // area of triangle
      sumNormal += area * normalVector; // area-weighted normal vector of triangle

      h = h_twin->next(); // next out-going half-edge of vertex

    } while(h != this->halfedge()); // keep going until back to same half-edge

    return sumNormal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (!e0->isBoundary()) {
      // inside
      HalfedgeIter h0 = e0->halfedge(); // current out-going half-edge
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin(); // twin of out-going halfedge
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      // outside
      HalfedgeIter h1_twin = h1->twin();
      HalfedgeIter h2_twin = h2->twin();
      HalfedgeIter h4_twin = h4->twin();
      HalfedgeIter h5_twin = h5->twin();

      // edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      // vertices
      VertexIter v0 = h0->vertex(); // old edge
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->vertex(); // new edge
      VertexIter v3 = h5->vertex();

      // faces
      FaceIter f1 = h0->face(); // face of current out-going half-edge
      FaceIter f2 = h3->face(); // face of twin 

      // updates
      /**
            * For convenience, this method sets all of the
            * neighbors of this halfedge to the given values.
            * 
           void setNeighbors( HalfedgeIter next,
                              HalfedgeIter twin,
                              VertexIter vertex,
                              EdgeIter edge,
                              FaceIter face )
      **/
      // inside
      h0->setNeighbors(h1, h3, v2, e0, f1);
      h1->setNeighbors(h2, h5_twin, v3, e4, f1);
      h2->setNeighbors(h0, h1_twin, v1, e1, f1);
      h3->setNeighbors(h4, h0, v3, e0, f2);
      h4->setNeighbors(h5, h2_twin, v2, e2, f2);
      h5->setNeighbors(h3, h4_twin, v0, e3, f2);

      // outside
      h1_twin->setNeighbors(h1_twin->next(), h2, v2, e1, h1_twin->face());
      h2_twin->setNeighbors(h2_twin->next(), h4, v0, e2, h2_twin->face());
      h4_twin->setNeighbors(h4_twin->next(), h5, v3, e3, h4_twin->face());
      h5_twin->setNeighbors(h5_twin->next(), h1, v1, e4, h5_twin->face());

      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h2;
      e2->halfedge() = h4;
      e3->halfedge() = h5;
      e4->halfedge() = h1;

      // vertices
      v0->halfedge() = h5;
      v1->halfedge() = h2;
      v2->halfedge() = h4;
      v3->halfedge() = h1;

      // faces
      f1->halfedge() = h0;
      f2->halfedge() = h3;
    }

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (!e0->isBoundary()) {
      // inside
      HalfedgeIter h0 = e0->halfedge(); // current out-going half-edge
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin(); // twin of out-going halfedge
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      // outside
      HalfedgeIter h1_twin = h1->twin();
      HalfedgeIter h2_twin = h2->twin();
      HalfedgeIter h4_twin = h4->twin();
      HalfedgeIter h5_twin = h5->twin();

      // edges
      EdgeIter e1 = h1->edge();
      EdgeIter e2 = h2->edge();
      EdgeIter e3 = h4->edge();
      EdgeIter e4 = h5->edge();

      // vertices
      VertexIter v0 = h0->vertex(); // old edge
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex(); // new edge
      VertexIter v3 = h5->vertex();

      // faces
      FaceIter f1 = h0->face(); // face of current out-going half-edge
      FaceIter f2 = h3->face(); // face of twin 

      // updates
      /**
          * For convenience, this method sets all of the
          * neighbors of this halfedge to the given values.
          * 
          void setNeighbors( HalfedgeIter next,
                             HalfedgeIter twin,
                             VertexIter vertex,
                             EdgeIter edge,
                             FaceIter face )
      **/
      // inside
      HalfedgeIter h6 = newHalfedge();
      HalfedgeIter h7 = newHalfedge();
      HalfedgeIter h8 = newHalfedge();
      HalfedgeIter h9 = newHalfedge();
      HalfedgeIter h10 = newHalfedge();
      HalfedgeIter h11 = newHalfedge();

      // edges
      EdgeIter e5 = newEdge();
      EdgeIter e6 = newEdge();
      EdgeIter e7 = newEdge();

      // vertex
      VertexIter v4 = newVertex();

      // faces
      FaceIter f3 = newFace();
      FaceIter f4 = newFace();

      // top-left
      h0->setNeighbors(h1, h3, v4, e0, f1);
      h1->setNeighbors(h2, h1_twin, v1, e1, f1);
      h2->setNeighbors(h0, h7, v2, e5, f1);

      // top-right
      h3->setNeighbors(h4, h0, v1, e0, f2);
      h4->setNeighbors(h5, h11, v4, e7, f2);
      h5->setNeighbors(h3, h5_twin, v3, e4, f2);

      // bottom-left
      h6->setNeighbors(h7, h9, v0, e6, f3);
      h7->setNeighbors(h8, h2, v4, e5, f3);
      h8->setNeighbors(h6, h2_twin, v2, e2, f3);

      // bottom-right
      h9->setNeighbors(h10, h6, v4, e6, f4);
      h10->setNeighbors(h11, h4_twin, v0, e3, f4);
      h11->setNeighbors(h9, h4, v3, e7, f4);

      // outside
      h1_twin->setNeighbors(h1_twin->next(), h1, v2, e1, h1_twin->face());
      h2_twin->setNeighbors(h2_twin->next(), h8, v0, e2, h2_twin->face());
      h4_twin->setNeighbors(h4_twin->next(), h10, v3, e3, h4_twin->face());
      h5_twin->setNeighbors(h5_twin->next(), h5, v1, e4, h5_twin->face());

      // new vertex
      v4->position = 0.5 * (v0->position + v1->position);
      v4->isNew = 1;

      // vertices
      v0->halfedge() = h6;
      v1->halfedge() = h1;
      v2->halfedge() = h8;
      v3->halfedge() = h5;
      v4->halfedge() = h0;

      // edges
      e0->halfedge() = h0;
      e1->halfedge() = h1;
      e2->halfedge() = h8;
      e3->halfedge() = h10;
      e4->halfedge() = h5;
      e5->halfedge() = h2;
      e6->halfedge() = h6;
      e7->halfedge() = h4;

      e0->isNew = 0;
      e6->isNew = 0;
      e5->isNew = 1;
      e7->isNew = 1;

      // faces
      f1->halfedge() = h0;
      f2->halfedge() = h3;
      f3->halfedge() = h6;
      f4->halfedge() = h9;

      return v4;
    }

    return VertexIter();
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->isNew = 0; // mark as vertex of original mesh

      HalfedgeIter h = v->halfedge();
      Vector3D neighbor_vertices_sum(0, 0, 0);
      do { // get the neighbor vertex positions
        neighbor_vertices_sum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      double n = v->degree();
      double u = (n == 3 ? (3.0 / 16.0) : (3.0 / (8.0 * n))); 
      v->newPosition = ((1 - n * u) * v->position) + (u * neighbor_vertices_sum); // new position of old vertex = 
                                                                                  // weighted current vertex position + weighted sum of neighbor vertex positions 
    }
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      e->isNew = 0; // mark as edge of original mesh

      // inside
      HalfedgeIter h0 = e->halfedge(); // current out-going half-edge
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin(); // twin of out-going halfedge
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();

      // vertices
      VertexIter v0 = h0->vertex(); // old edge
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h2->vertex(); // across old edge
      VertexIter v3 = h5->vertex();

      e->newPosition = 3.0 / 8.0 * (v0->position + v1->position) + 1.0 / 8.0 * (v2->position + v3->position); // new midpoint vertex =
                                                                                                              // weighted vertices of current edge + weighted vertices across current edge
    }
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)

    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v0 = e->halfedge()->vertex(); // bottom vertex of edge
      VertexIter v1 = e->halfedge()->twin()->vertex(); // top vertex of edge

      if (!(v0->isNew || v1->isNew)) {
        VertexIter v = mesh.splitEdge(e);
        v->newPosition = e->newPosition; // new position of vertex
      }
    }
    
    // 4. Flip any new edge that connects an old and new vertex.
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      VertexIter v0 = e->halfedge()->vertex(); // bottom vertex of edge
      VertexIter v1 = e->halfedge()->twin()->vertex(); // top vertex of edge

      if (e->isNew && ((v0->isNew && !v1->isNew) || (!v0->isNew && v1->isNew))) {
        mesh.flipEdge(e);
      }
    }

    // 5. Copy the new vertex positions into final Vertex::position.
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
      v->isNew = 0;
    }
  }
}
