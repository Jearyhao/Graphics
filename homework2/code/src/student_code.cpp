#include "student_code.h"
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
    std::vector<Vector2D> resPoints;
    for (int i = 0; i < points.size() - 1; i++)
    {
      Vector2D p1 = points[i];
      Vector2D p2 = points[i + 1];
      Vector2D newPoint = (1 - t) * p1 + t * p2;
      resPoints.push_back(newPoint);
    }
    
    return resPoints;
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
	  std::vector<Vector3D> resPoints;
      for (int i = 0; i < points.size() - 1; i++) {
          Vector3D p1 = points[i];
          Vector3D p2 = points[i + 1];
		  Vector3D newPoint = (1 - t) * p1 + t * p2;
		  resPoints.push_back(newPoint);
      }
	  return resPoints;
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
	  vector<Vector3D> resPoints = BezierPatch::evaluateStep(points, t);
	  while (resPoints.size() > 1) {
		  resPoints = BezierPatch::evaluateStep(resPoints, t);
	  }
	  return resPoints[0];
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
	  vector<Vector3D> points;
	  for (int i = 0; i < controlPoints.size(); i++) {
		  points.push_back(BezierPatch::evaluate1D(controlPoints[i], u));
	  }
	  return BezierPatch::evaluate1D(points, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      // ��ȡ��ö��������İ��
      HalfedgeCIter h = halfedge();

      // ��ʼ��������
      Vector3D normal(0, 0, 0);

      // �����������ڵ�������
      do {
          // ��ȡ��ǰ�����ε���������
          Vector3D p0 = h->vertex()->position;
          Vector3D p1 = h->next()->vertex()->position;
          Vector3D p2 = h->next()->next()->vertex()->position;

          // ���㵱ǰ�����εķ�����
          Vector3D faceNormal = cross(p1 - p0, p2 - p0);

          // ���㵱ǰ�����ε����
          double area = faceNormal.norm() / 2.0;

          // ����������Ȩ�ۼӵ��ܷ�����
          normal += faceNormal * area;

          // �ƶ�����һ�����ڵ�������
          h = h->twin()->next();
      } while (h != halfedge());

      // ��һ��������
      return normal.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      // ��ȡ���������ص����а�ߡ����㡢�ߺ���
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->twin();
      if (e0->isBoundary()){
          return e0;
      }

      HalfedgeIter h2 = h0->next();
      HalfedgeIter h3 = h2->next();

      HalfedgeIter h4 = h1->next();
      HalfedgeIter h5 = h4->next();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h3->vertex();
      VertexIter v3 = h5->vertex();

      FaceIter f0 = h0->face();
      FaceIter f1 = h1->face();

	  // ���°�ߵ�next
      h0->next() = h5;
      h5->next() = h2;
      h2->next() = h0;
	  h1->next() = h3;
	  h3->next() = h4;
      h4->next() = h1;
      //���°�ߵ�twin
      h0->twin() = h1;
      h1->twin() = h0;
      //���°�ߵ�vertex
      h0->vertex() = v2;
      h1->vertex() = v3;
      h2->vertex() = v1;
      h3->vertex() = v2;
	  h4->vertex() = v0;
      h5->vertex() = v3;
      //���°�ߵ�edge
	  h0->edge() = e0;
      h1->edge() = e0;
	  //���°�ߵ�face
	  h0->face() = f0;
      h2->face() = f0;
      h5->face() = f0;
      h1->face() = f1;
      h3->face() = f1;
      h4->face() = f1;

	  //���¶����halfedge
	  v0->halfedge() = h4;
      v1->halfedge() = h2;
	  v2->halfedge() = h0;
      v3->halfedge() = h5;

	  //���±ߵ�halfedge
	  e0->halfedge() = h0;

	  //�������halfedge
	  f0->halfedge() = h0;
      f1->halfedge() = h1;
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    // ��ȡ���������ص����а�ߡ����㡢�ߺ���
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->twin();
      HalfedgeIter h2 = h0->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h1->next();
      HalfedgeIter h5 = h4->next();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h3->vertex();
      VertexIter v3 = h5->vertex();

      FaceIter f0 = h0->face();
      FaceIter f1 = h1->face();
      if (e0->isBoundary()) {
         VertexIter vNew = newVertex();
         EdgeIter eNew1 = newEdge();
         EdgeIter eNew2 = newEdge();
         HalfedgeIter hNew1 = newHalfedge();
         HalfedgeIter hNew2 = newHalfedge();
         HalfedgeIter hNew3 = newHalfedge();
         HalfedgeIter hNew4 = newHalfedge();
         FaceIter fNew = newFace();
         vNew->isNew = true;
         eNew1->isNew = true;
         vNew->position = (v0->position + v1->position) / 2.0;
         // ���°�ߵ�next
         h0->next() = h2;
         h2->next() = hNew1;
         hNew1->next() = h0;

         hNew2->next() = h3;
         h3->next() = hNew3;
         hNew3->next() = hNew2;

         h1->next() = hNew4;
         hNew4->next() = h4;

         //���°�ߵ�twin
         hNew1->twin() = hNew2;
         hNew2->twin() = hNew1;
         hNew3->next() = hNew4;

         //���°�ߵĶ���
         h0->vertex() = vNew;
         hNew1->vertex() = v2;
         hNew2->vertex() = vNew;
         hNew3->vertex() = v0;
         hNew4->vertex() = vNew;

         //���°�ߵı�
         h0->edge() = e0;
         h1->edge() = e0;
         hNew1->edge() = eNew1;
         hNew2->edge() = eNew1;
         hNew3->edge() = eNew2;
         hNew4->edge() = eNew2;

         //���°�ߵ���
         h0->face() = f0;
         h2->face() = f0;
         hNew1->face() = f0;
         hNew2->face() = fNew;
         h3->face() = fNew;
         hNew3->face() = fNew;
         hNew4->face() = h1->face();

         //���¶���İ��
         vNew->halfedge() = h0;
         v1->halfedge() = h2;
         v2->halfedge() = hNew1;
         v0->halfedge() = hNew3;

         // ���±ߵİ��
         e0->halfedge() = h0;
         eNew1->halfedge() = hNew1;
         eNew2->halfedge() = hNew2;

         // ������İ��
         f0->halfedge() = h0;
         fNew->halfedge() = hNew2;

         return vNew;
      } else{
        // �����µĶ��㡢��ߡ��ߺ���
        VertexIter vNew = newVertex();
        EdgeIter eNew1 = newEdge();
        EdgeIter eNew2 = newEdge();
        EdgeIter eNew3 = newEdge();
        HalfedgeIter hNew1 = newHalfedge();
        HalfedgeIter hNew2 = newHalfedge();
        HalfedgeIter hNew3 = newHalfedge();
        HalfedgeIter hNew4 = newHalfedge();
        HalfedgeIter hNew5 = newHalfedge();
        HalfedgeIter hNew6 = newHalfedge();
        FaceIter fNew1 = newFace();
        FaceIter fNew2 = newFace();
		eNew1->isNew = true;
        eNew3->isNew = true;
        // �����¶����λ��
        //vNew->position = (v0->position + v1->position) / 2.0;

        // ���°�ߵ�next
        h0->next() = h2;
        h2->next() = hNew1;
        hNew1->next() = h0;

        h1->next() = hNew3;
        hNew3->next() = h5;
        h5->next() = h1;

        hNew2->next() = h3;
        h3->next() = hNew5;
        hNew5->next() = hNew2;

        hNew4->next() = hNew6;
        hNew6->next() = h4;
        h4->next() = hNew4;

      //���°�ߵ�twin
        hNew1->twin() = hNew2;
        hNew2->twin() = hNew1;
        hNew3->twin() = hNew4;
        hNew4->twin() = hNew3;
        hNew5->twin() = hNew6;
        hNew6->twin() = hNew5;

        //���°�ߵĶ���
        h0->vertex() = vNew;
        h1->vertex() = v1;
        h2->vertex() = v1;
        h3->vertex() = v2;
        h4->vertex() = v0;
        h5->vertex() = v3;
        hNew1->vertex() = v2;
        hNew2->vertex() = vNew;
        hNew3->vertex() = vNew;
        hNew4->vertex() = v3;
        hNew5->vertex() = v0;
        hNew6->vertex() = vNew;

      //���°�ߵı�
        hNew1->edge() = eNew1;
        hNew2->edge() = eNew1;
        hNew5->edge() = eNew2;
        hNew6->edge() = eNew2;
        hNew3->edge() = eNew3;
        hNew4->edge() = eNew3;

      //���°�ߵ���
        hNew1->face() = f0;
        hNew2->face() = fNew1;
        hNew5->face() = fNew1;
        hNew6->face() = fNew2;
        hNew4->face() = fNew2;
        hNew3->face() = f1;
        h0->face() = f0;
        h1->face() = f1;
		h2->face() = f0;
        h3->face() = fNew1;
        h4->face() = fNew2;
        h5->face() = f1;

        //���¶���İ��
        vNew->halfedge() = h0;
        v2->halfedge() = hNew1;
        v0->halfedge() = hNew5;
        v1->halfedge() = h2;
        v3->halfedge() = hNew4;
        
        // ���±ߵİ��
        e0->halfedge() = h0;
        eNew1->halfedge() = hNew1;
        eNew2->halfedge() = hNew5;
        eNew3->halfedge() = hNew3;

        // ������İ��
        f0->halfedge() = h0;
        f1->halfedge() = h1;
        fNew1->halfedge() = h3;
        fNew2->halfedge() = h4;

        return vNew;
      }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
        // TODO Part 6.
        // 1. �����������������ж������λ�ã�ʹ�� Loop ϸ�ֹ��򣬲�����洢�� Vertex::newPosition �С�
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
          // ��ȡ���� v �������ھӶ���
          std::vector<VertexIter> neighbors;
          HalfedgeIter h = v->halfedge();
          do {
              neighbors.push_back(h->twin()->vertex());
              h = h->twin()->next();
          } while (h != v->halfedge());

          // ����Ȩ��ϵ��
          int n = neighbors.size();
          double u;
          if (n == 3) {
              u = 3.0 / 16.0;
          }
          else {
              u = 3.0 / (8.0 * n);
          }

          // ʹ�� Loop ϸ�ֹ�������¶���λ��
          Vector3D newP = (1.0 - n * u) * v->position;
          for (VertexIter neighbor : neighbors) {
              newP += u * neighbor->position;
          }
          v->newPosition = newP;
          v->isNew = false;
      }

      // 2. ���������صĸ��¶���λ�ã�������洢�� Edge::newPosition �С�
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
          // ��ȡ��� e ��ص��ĸ�����
          HalfedgeIter h0 = e->halfedge();
          HalfedgeIter h1 = h0->twin();

          VertexIter v0 = h0->vertex();
          VertexIter v1 = h1->vertex();
          VertexIter v2 = h0->next()->next()->vertex();
          VertexIter v3 = h1->next()->next()->vertex();

          // ʹ�� Loop ϸ�ֹ������ߵ���λ��
          Vector3D newPosition = (3.0 / 8.0) * (v0->position + v1->position) + (1.0 / 8.0) * (v2->position + v3->position);
          e->newPosition = newPosition;
          //e->isNew = false; // ���Ϊԭʼ��
      }

      // 3. ��������е�ÿ���ߣ�������±�
      std::vector<EdgeIter> edges;
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
		  edges.push_back(e);
      }

      for (EdgeIter e : edges) {
          VertexIter newVertex = mesh.splitEdge(e);
		  newVertex->position = e->newPosition;
          newVertex->isNew = true;
      }

      // 4. ��ת�κ����Ӿɶ�����¶�����±�
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
          if (e->isNew) {
              VertexIter v0 = e->halfedge()->vertex();
              VertexIter v1 = e->halfedge()->twin()->vertex();
              if ((v0->isNew && !v1->isNew) || (!v0->isNew && v1->isNew)) {
                    mesh.flipEdge(e);
              }
          }
      }

      //5. ���ɶ���λ�ø��Ƶ����յ� Vertex::position ��
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
		  if (!v->isNew) {
			  v->position = v->newPosition;
		  }
      }
	  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
		  e->isNew = false;
	  }
	  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
		  v->isNew = false;
	  }
  }
}
