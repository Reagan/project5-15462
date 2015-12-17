/*
 * Student solution for CMU 15-462 Project 2 (MeshEdit)
 *
 * Implemented by ____ on ____.
 *
 */

#include "student_code.h"
#include "mutablePriorityQueue.h"
#include <iostream>
#include <vector>
#include <algorithm>

namespace CMU462
{
    EdgeIter HalfedgeMesh::flipEdge( EdgeIter selectedEdge )
    {

       if (selectedEdge->isBoundary() || selectedEdge->halfedge()->twin()->edge()->isBoundary()) {
          cout << "Error! Selected boundary edge. Cannot flip.";
          return HalfedgeMesh::edgesEnd() ;
       }

       //******************************************
       // 1. Get pointers to the various elements
       //******************************************
       // get the half edges

       HalfedgeIter dbHE = selectedEdge->halfedge() ;
       HalfedgeIter bdHE = dbHE->twin() ;
       HalfedgeIter baHE = dbHE->next() ;
       HalfedgeIter abHE = baHE->twin() ;
       HalfedgeIter adHE = baHE->next() ;
       HalfedgeIter daHE = adHE->twin() ;
       HalfedgeIter dcHE = bdHE->next() ;
       HalfedgeIter cdHE = dcHE->twin() ;
       HalfedgeIter cbHE = dcHE->next() ;
       HalfedgeIter bcHE = cbHE->twin() ;

       // get vertices
       VertexIter aV = adHE->vertex() ;
       VertexIter bV = bdHE->vertex();
       VertexIter cV = cbHE->vertex();
       VertexIter dV = dbHE->vertex();

       // get edges
       EdgeIter baEdge = baHE->edge() ;
       EdgeIter adEdge = adHE->edge() ;
       EdgeIter bdEdge = bdHE->edge() ;
       EdgeIter dcEdge = dcHE->edge() ;
       EdgeIter cbEdge = cbHE->edge() ;

       // get faces
       FaceIter f1 = dbHE->face() ;
       FaceIter f2 = bdHE->face() ;

       //**************************
       // 2. Reallocate pointers
       //**************************
       // set half edges
       abHE->next() = abHE->next() ;
       abHE->twin() = adHE;
       abHE->vertex() = aV;
       abHE->edge() = baEdge;
       abHE->face() = abHE->face() ;

       baHE->next() = adHE ;
       baHE->twin() = bcHE;
       baHE->vertex() = cV;
       baHE->edge() = cbEdge;
       baHE->face() = f1;

       adHE->next() = dbHE ;
       adHE->twin() = abHE;
       adHE->vertex() = bV;
       adHE->edge() = baEdge;
       adHE->face() = f1;

       daHE->next() = daHE->next() ;
       daHE->twin() = dcHE;
       daHE->vertex() = dV;
       daHE->edge() = adEdge;
       daHE->face() = daHE->face() ;

       cdHE->next() = cdHE->next() ;
       cdHE->twin() = cbHE;
       cdHE->vertex() = cV;
       cdHE->edge() = dcEdge;
       cdHE->face() = cdHE->face() ;

       dcHE->next() = cbHE ;
       dcHE->twin() = daHE;
       dcHE->vertex() = aV;
       dcHE->edge() = adEdge;
       dcHE->face() = f2;

       bcHE->next() = bcHE->next() ;
       bcHE->twin() = baHE;
       bcHE->vertex() = bV;
       bcHE->edge() = cbEdge;
       bcHE->face() = bcHE->face() ;

       cbHE->next() = bdHE ;
       cbHE->twin() = cdHE;
       cbHE->vertex() = dV;
       cbHE->edge() = dcEdge;
       cbHE->face() = f2;

       dbHE->next() = baHE ;
       dbHE->twin() = bdHE;
       dbHE->vertex() = aV;
       dbHE->edge() = bdEdge;
       dbHE->face() = f1;

       bdHE->next() = dcHE ;
       bdHE->twin() = dbHE;
       bdHE->vertex() = cV;
       bdHE->edge() = bdEdge;
       bdHE->face() = f2;

       // set vertices
       aV->halfedge() = dbHE ;
       bV->halfedge() = adHE ;
       cV->halfedge() = bdHE ;
       dV->halfedge() = cbHE ;

       // set edges
       baEdge->halfedge() = adHE ;
       bdEdge->halfedge() = bdHE ;
       adEdge->halfedge() = dcHE ;
       dcEdge->halfedge() = cbHE ;
       cbEdge->halfedge() = baHE ;

       // set faces
       f1->halfedge() = dbHE ;
       f2->halfedge() = bdHE ;

       return HalfedgeMesh::edgesBegin() ;
    }

    VertexIter HalfedgeMesh::splitEdge( EdgeIter selectedEdge )
    {
       // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
       // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.
       if (selectedEdge->isBoundary() || selectedEdge->halfedge()->twin()->edge()->isBoundary()) {
          cout << "Error! Selected boundary edge. Cannot flip.";
          return HalfedgeMesh::newVertex() ;
       }

       //******************************************
       // 1. Get pointers to the various elements
       //******************************************
       // get the half edges

       HalfedgeIter dbHE = selectedEdge->halfedge() ;
       HalfedgeIter bdHE = dbHE->twin() ;
       HalfedgeIter baHE = dbHE->next() ;
       HalfedgeIter abHE = baHE->twin() ;
       HalfedgeIter adHE = baHE->next() ;
       HalfedgeIter daHE = adHE->twin() ;
       HalfedgeIter dcHE = bdHE->next() ;
       HalfedgeIter cdHE = dcHE->twin() ;
       HalfedgeIter cbHE = dcHE->next() ;
       HalfedgeIter bcHE = cbHE->twin() ;

       // get vertices
       VertexIter aV = adHE->vertex() ;
       VertexIter bV = bdHE->vertex();
       VertexIter cV = cbHE->vertex();
       VertexIter dV = dbHE->vertex();

       // get edges
       EdgeIter baEdge = baHE->edge() ;
       EdgeIter adEdge = adHE->edge() ;
       EdgeIter bdEdge = bdHE->edge() ;
       EdgeIter dcEdge = dcHE->edge() ;
       EdgeIter cbEdge = cbHE->edge() ;

       // get faces
       FaceIter f1 = dbHE->face() ;
       FaceIter f2 = bdHE->face() ;

       //******************************************
       // 2. Create new required elements
       //******************************************

       // create new vertex at mid position
       VertexIter mV = HalfedgeMesh::newVertex() ;
       Vector3D v1Position = selectedEdge->halfedge()->vertex()->position ;
       Vector3D v2Position = selectedEdge->halfedge()->twin()->vertex()->position ;
       Vector3D middlePosition( (float) (v2Position.x + v1Position.x) / 2,
                                (float) (v2Position.y + v1Position.y) / 2,
                                (float) (v2Position.z + v1Position.z) / 2 ) ;
       mV->position = middlePosition;

       // create new edges
       EdgeIter amEdge = HalfedgeMesh::newEdge() ;
       EdgeIter dmEdge = HalfedgeMesh::newEdge() ;
       EdgeIter mcEdge = HalfedgeMesh::newEdge() ;

       // create new faces
       FaceIter f3 = HalfedgeMesh::newFace() ;
       FaceIter f4 = HalfedgeMesh::newFace() ;

       // create new half edges
       HalfedgeIter amHE = HalfedgeMesh::newHalfedge() ;
       HalfedgeIter maHE = HalfedgeMesh::newHalfedge() ;
       HalfedgeIter dmHE = HalfedgeMesh::newHalfedge() ;
       HalfedgeIter mcHE = HalfedgeMesh::newHalfedge() ;
       HalfedgeIter cmHE = HalfedgeMesh::newHalfedge() ;
       HalfedgeIter mdHE = HalfedgeMesh::newHalfedge() ;

       //******************************************
       // 3. Reassign pointers to new elements
       //******************************************
       // set half edges
       abHE->next() = abHE->next() ;
       abHE->twin() = baHE;
       abHE->vertex() = aV;
       abHE->edge() = baEdge;
       abHE->face() = abHE->face() ;

       baHE->next() = amHE ;
       baHE->twin() = abHE;
       baHE->vertex() = bV;
       baHE->edge() = baEdge;
       baHE->face() = f3;

       adHE->next() = dmHE ;
       adHE->twin() = daHE;
       adHE->vertex() = aV;
       adHE->edge() = adEdge;
       adHE->face() = f2;

       daHE->next() = daHE->next() ;
       daHE->twin() = adHE;
       daHE->vertex() = dV;
       daHE->edge() = adEdge;
       daHE->face() = daHE->face() ;

       cdHE->next() = cdHE->next() ;
       cdHE->twin() = dcHE;
       cdHE->vertex() = cV;
       cdHE->edge() = dcEdge;
       cdHE->face() = cdHE->face() ;

       dcHE->next() = cmHE ;
       dcHE->twin() = cdHE;
       dcHE->vertex() = dV;
       dcHE->edge() = dcEdge;
       dcHE->face() = f4;

       bcHE->next() = bcHE->next() ;
       bcHE->twin() = cbHE;
       bcHE->vertex() = bV;
       bcHE->edge() = cbEdge;
       bcHE->face() = bcHE->face() ;

       cbHE->next() = bdHE ;
       cbHE->twin() = bcHE;
       cbHE->vertex() = cV;
       cbHE->edge() = cbEdge;
       cbHE->face() = f1;

       dbHE->next() = baHE ;
       dbHE->twin() = bdHE;
       dbHE->vertex() = mV;
       dbHE->edge() = bdEdge;
       dbHE->face() = f3;

       bdHE->next() = mcHE ;
       bdHE->twin() = dbHE;
       bdHE->vertex() = bV;
       bdHE->edge() = bdEdge;
       bdHE->face() = f1;

       // --> additional vertices
       amHE->next() = dbHE ;
       amHE->twin() = maHE ;
       amHE->vertex() = aV ;
       amHE->edge() = amEdge ;
       amHE->face() = f3 ;

       maHE->next() =  adHE ;
       maHE->twin() = amHE ;
       maHE->vertex() = mV ;
       maHE->edge() = amEdge ;
       maHE->face() = f2 ;

       dmHE->next() = maHE ;
       dmHE->twin() = mdHE ;
       dmHE->vertex() = dV ;
       dmHE->edge() = dmEdge ;
       dmHE->face() = f2 ;

       mcHE->next() = cbHE ;
       mcHE->twin() = cmHE ;
       mcHE->vertex() = mV ;
       mcHE->edge() = mcEdge ;
       mcHE->face() = f1 ;

       cmHE->next() = mdHE ;
       cmHE->twin() = mcHE ;
       cmHE->vertex() = cV ;
       cmHE->edge() = mcEdge ;
       cmHE->face() = f4 ;

       mdHE->next() = dcHE ;
       mdHE->twin() = dmHE ;
       mdHE->vertex() = mV ;
       mdHE->edge() = dmEdge ;
       mdHE->face() = f4 ;

       // set vertices
       aV->halfedge() = amHE ;
       bV->halfedge() = bdHE ;
       cV->halfedge() = cmHE ;
       dV->halfedge() = dmHE ;

       // additional vertices
       mV->halfedge() = dbHE ;

       // set edges
       baEdge->halfedge() = baHE ;
       bdEdge->halfedge() = bdHE ;
       adEdge->halfedge() = adHE ;
       dcEdge->halfedge() = dcHE ;
       cbEdge->halfedge() = cbHE ;

       // additional edges
       amEdge->halfedge() = amHE ;
       dmEdge->halfedge() = dmHE ;
       mcEdge->halfedge() = mcHE ;

       // set faces
       f1->halfedge() = mcHE ;
       f2->halfedge() = dmHE ;

       // additional faces
       f3->halfedge() = amHE ;
       f4->halfedge() = mdHE ;

       return mV;
    }

    VertexIter HalfedgeMesh::collapseEdge( EdgeIter selectedEdge )
    {
       if (selectedEdge->isBoundary() || selectedEdge->halfedge()->twin()->edge()->isBoundary()) {
          cout << "Error! Selected boundary edge. Cannot flip.";
          return HalfedgeMesh::newVertex() ;
       }

       // TODO This method should collapse the given edge and return an iterator to the new vertex created by the collapse.
       //***********************************************
       // 1. Get half edges connected to selected edge
       //***********************************************

       HalfedgeIter dbHE = selectedEdge->halfedge() ;
       HalfedgeIter bdHE = dbHE->twin() ;
       HalfedgeIter baHE = dbHE->next() ;
       HalfedgeIter abHE = baHE->twin() ;
       HalfedgeIter adHE = baHE->next() ;
       HalfedgeIter daHE = adHE->twin() ;
       HalfedgeIter dcHE = bdHE->next() ;
       HalfedgeIter cdHE = dcHE->twin() ;
       HalfedgeIter cbHE = dcHE->next() ;
       HalfedgeIter bcHE = cbHE->twin() ;

       // get vertices
       VertexIter aV = adHE->vertex() ;
       VertexIter bV = bdHE->vertex();
       VertexIter cV = cbHE->vertex();
       VertexIter dV = dbHE->vertex();

       // get edges
       EdgeIter baEdge = baHE->edge() ;
       EdgeIter adEdge = adHE->edge() ;
       EdgeIter bdEdge = bdHE->edge() ;
       EdgeIter dcEdge = dcHE->edge() ;
       EdgeIter cbEdge = cbHE->edge() ;

       // get faces
       FaceIter f1 = dbHE->face() ;
       FaceIter f2 = bdHE->face() ;

       // create required components
       VertexIter mV = newVertex() ;
       mV->halfedge() = halfedges.end();
       Vector3D v1Position = selectedEdge->halfedge()->vertex()->position ;
       Vector3D v2Position = selectedEdge->halfedge()->twin()->vertex()->position ;
       Vector3D middlePosition( (float) (v2Position.x + v1Position.x) / 2,
                                (float) (v2Position.y + v1Position.y) / 2,
                                (float) (v2Position.z + v1Position.z) / 2 ) ;
       mV->position = middlePosition;

       cout << "mV address: " << elementAddress(mV) << endl ;

       EdgeIter amEdge = newEdge() ;
       EdgeIter cmEdge = newEdge() ;

       // processing
       HalfedgeIter e = dbHE ;

       cout << "dbHE: " << elementAddress(dbHE) << endl ;
       do {
          e = e->twin() ;
          if (e != dbHE && e != bdHE && e != baHE && e != adHE
              && e != dcHE && e!= cbHE) {
             cout << "in " << elementAddress(e) << endl;
          }

          e = e->next() ;
          if (e != dbHE && e != bdHE && e != baHE && e != adHE
              && e != dcHE && e!= cbHE) {

             e->vertex() = mV ;

             cout << "out " << elementAddress(e) << endl;
          }
       }
       while( e != dbHE );

       HalfedgeIter f = bdHE ;

       cout << "bdHE: " << elementAddress(bdHE) << endl ;
       do {
          f = f->twin() ;
          if (f != dbHE && f != bdHE && f != baHE && f != adHE
              && f != dcHE && f != cbHE) {
             cout << "in2 " << elementAddress(f) << endl;
          }

          f = f->next() ;
          if (f != dbHE && f != bdHE && f != baHE && f != adHE
              && f != dcHE && f != cbHE) {
             f->vertex() = mV ;
             cout << "out2 " << elementAddress(f) << endl;
          }
       }
       while( f != bdHE );


       cdHE->twin() = bcHE ;
       cdHE->edge() = cmEdge ;

       daHE->twin() = abHE ;
       daHE->edge() = amEdge ;
       daHE->vertex() = mV ;

       abHE->twin() = daHE ;
       abHE->edge() = amEdge ;

       bcHE->twin() = cdHE ;
       bcHE->edge() = cmEdge ;
       bcHE->vertex() = mV ;

       amEdge->halfedge()  = abHE ;
       cmEdge->halfedge() = cdHE ;
       mV->halfedge() = daHE ;
       aV->halfedge() = abHE ;
       cV->halfedge() = cdHE ;

       // delete unnecessary mesh elements
       deleteHalfedge(dbHE) ;
       deleteHalfedge(bdHE) ;
       deleteHalfedge(baHE) ;
       deleteHalfedge(adHE) ;
       deleteHalfedge(dcHE) ;
       deleteHalfedge(cbHE) ;

       deleteEdge(adEdge) ;
       deleteEdge(dcEdge) ;
       deleteEdge(cbEdge) ;
       deleteEdge(baEdge) ;
       deleteEdge(bdEdge) ;

       deleteFace(f1) ;
       deleteFace(f2) ;

       deleteVertex(bV) ;
       deleteVertex(dV) ;

       return mV ;
    }

    HalfedgeMesh* MeshResampler::catmullClarkSubdivide( HalfedgeMesh& mesh, int rounds )
    {
         cout <<  "Catmull clark resample called [" << rounds << "]" << endl ;
         vector<FaceCIter> processedFaces ;

          // ***************************************************
          // Step 1: find the new FACE points
          // ***************************************************
          for (FaceIter vIter = mesh.facesBegin() ; vIter != mesh.facesEnd(); vIter++) {

          }

         // ***************************************************
         // Step 1: find the new FACE points
         // ***************************************************

         for (VertexIter vIter = mesh.verticesBegin() ; vIter != mesh.verticesEnd(); vIter++) {
            // get the faces
            // mark as belonging to the old mesh
            vIter->isNew = false ;

            // loop through all half edges
            HalfedgeIter h = vIter->halfedge(); // get one of the outgoing half-edges of the vertex
            do
            {
               // get the half edge's face
               FaceIter currFace = h->face();

               // find if the current face has already been processed
               vector <FaceCIter>::iterator i = processedFaces.begin ();
               i = find(processedFaces.begin (), processedFaces.end(), currFace);

               if (i == processedFaces.end ())
               {
                  cout << "face not found. processing..." << endl ;
                  // ** perform processing **//
                  // get the next 2 vertices
                  VertexCIter vNext = h->next()->next()->vertex() ;
                  VertexCIter vNextNext = h->next()->next()->next()->vertex() ;

                  // get the 3 vertices
                  Vector3D firstVertexLocation = vIter->position ;
                  Vector3D secondVertexLocation = vNext->position ;
                  Vector3D thirdVertexLocation = vNextNext->position ;

                  // get the mean of the location
                  Vector3D meanLocation;
                  //((firstVertexLocation + secondVertexLocation + thirdVertexLocation) /3) ;

                  // append to mean locations
                  currFace->centerPoint = meanLocation ;

                  // add face to set of processed faces
                  processedFaces.push_back(currFace) ;
               }
               else
               {
                  // face already processed
                  int nPosition = distance (processedFaces.begin (), i);
                  cout << "face found in the vector at position: " << nPosition << ". exiting" << endl;
               }

               // move to next vertex
               HalfedgeIter h_twin = h->twin(); // get the vertex of the current halfedge
               VertexIter v = h_twin->vertex(); // vertex is 'source' of the half edge.
               h = h_twin->next(); // move to the next outgoing half-edge of the vertex.
            } while( h != vIter->halfedge() ); // keep going until we're back at the beginning
         }

          cout << endl ;
          int i=0;

          // ***************************************************
          // Step 2: find the new EDGE points
          // ***************************************************
          cout << endl ;
          i=0;
          for (EdgeIter vIter = mesh.edgesBegin() ; vIter != mesh.edgesEnd(); vIter++) {

             cout << "computing average for edge: " << i <<  " " << endl ; i++;

             // get the location for the current vertex
             Vector3D currVertexPos = vIter->halfedge()->vertex()->position ;

             // get the next vertex
             Vector3D vIterNextPos = vIter->halfedge()->next()->vertex()->position ;

             // get the current edge's face' face point
             Vector3D currEdgeFaceCenterPoint = vIter->halfedge()->face()->centerPoint ;

             // get the twin's half edge face's face point
             Vector3D twinHalfEdgeFaceCenterPoint = vIter->halfedge()->twin()->face()->centerPoint ;

             // get the mean of all locations
             vIter->newPosition = (currVertexPos + vIterNextPos + currEdgeFaceCenterPoint + twinHalfEdgeFaceCenterPoint) / 4 ;
          }
         cout << endl ;

       // *******************************************************
       // Step 3: find the new VERTEX/CONTROL points
       // (a) Since we are using a triangular mesh, the valence
       // is 4 (4 edges from each vertex)
       // (b) From the equation given for in the Catmull-Clark
       // paper, (Q/n) + (2R/n) + (S(n-3)/n), where n is valence
       // simplifies to (Q/4) + (R/2) + (S/4)
       // *******************************************************
       i=0;
       for (VertexIter vIter = mesh.verticesBegin() ; vIter != mesh.verticesEnd(); vIter++) {
          cout << "computing midpoints for edge " << i << endl ; i++;
          // get the faces
          // mark as belonging to the old mesh
          vIter->isNew = false;

          // find S (current vertice/control point)
          Vector3D s = vIter->position ;

          // find Q (average of sorrounding Q points)
          Vector3D firstControlPoint = vIter->halfedge()->face()->centerPoint ;
          Vector3D secondControlPoint = vIter->halfedge()->twin()->face()->centerPoint ;
          Vector3D thirdControlPoint = vIter->halfedge()->next()->next()->twin()->face()->centerPoint ;
          Vector3D fourthControlPoint = vIter->halfedge()->twin()->next()->twin()->face()->centerPoint ;

          Vector3D q = (firstControlPoint + secondControlPoint + thirdControlPoint + fourthControlPoint) / 4 ;

          // find R (average of sorrounding edge midpoints)
          Vector3D firstEdgeMidPoint = (s + vIter->halfedge()->next()->vertex()->position) / 2;
          Vector3D secondEdgeMidPoint = (s + vIter->halfedge()->twin()->next()->next()->vertex()->position) / 2;
          Vector3D thirdEdgeMidPoint = (s + vIter->halfedge()->next()->next()->vertex()->position) / 2;
          Vector3D fourthEdgeMidPoint = (s + vIter->halfedge()->next()->next()->twin()->next()->next()->vertex()->position) / 2;

          Vector3D r = (firstEdgeMidPoint + secondEdgeMidPoint + thirdEdgeMidPoint + fourthEdgeMidPoint) / 4 ;

          // implement the formula to get new position for vertex
          vIter->newPosition = q/4 + r/2 + s/4 ;
       }

       // ******************************************************
       // Step 4: Join the EDGE points to create the new mesh
       // ******************************************************

       // physcal modifications
       // connect face points to edge points
       HalfedgeMesh* newMesh ;

       // connect control points to edge points
       // uses from zero (no points for directly transform
       //
       // newMesh

       // dynamic additions
       // delete all components of mesh
       // delete faces
       for (FaceIter vIter = mesh.facesBegin() ; vIter != mesh.facesEnd(); vIter++) {
            mesh.deleteFace(vIter) ;
       }

       // delete edges
       for (EdgeIter vIter = mesh.edgesBegin() ; vIter != mesh.edgesEnd(); vIter++) {
            mesh.deleteEdge(vIter) ;
       }

       // delete vertices
       for (VertexIter vIter = mesh.verticesBegin() ; vIter != mesh.verticesEnd(); vIter++) {
         mesh.deleteVertex(vIter) ;
       }

       // delete half edges
       for (HalfedgeIter vIter = mesh.halfedgesBegin() ; vIter != mesh.halfedgesEnd(); vIter++) {
          mesh.deleteHalfedge(vIter) ;
       }

       // delete boundaries
       for (FaceIter vIter = mesh.boundariesBegin() ; vIter != mesh.boundariesEnd(); vIter++) {
          mesh.deleteFace(vIter) ;
       }

       // reassign mesh
       mesh = *newMesh ;

       // render mesh
       return &mesh ;

    }

    void MeshResampler::upsample( HalfedgeMesh& mesh )
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
       // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
       // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
       // using the connectivity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
       // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
       // the new mesh based on the values we computed for the original mesh.

       float weight;
       float sumx, sumy, sumz ;

       // mark a vertex as old and calculate new
       // position for it
       for (VertexIter vIter = mesh.verticesBegin() ; vIter != mesh.verticesEnd(); vIter++) {

          // mark as belonging to the old mesh
          vIter->isNew = false ;

          // compute updated positions for vertex
          Vector3D calculatedPosition ;

          // set u, the weight
          int nsize = vIter->degree() ;
          if (nsize == 3)
             weight = 3 * 0.1875 ; // nu
          else
             weight = nsize * 3/ (8 * nsize) ; // nu

          // set the position for the old vertices using formula
          // ( (1 - nu) + u * sum of positions of all neighbors )
          sumx = (1 - weight) * vIter->position.x ; // (1 - nu)
          sumy = (1 - weight) * vIter->position.y ;
          sumz = (1 - weight) * vIter->position.z ;

          // get the neighbors and add their location values
          // to additions
          HalfedgeCIter h = vIter->halfedge(); // get one of the outgoing halfedges of the vertex
          do {
             HalfedgeCIter h_twin = h->twin(); // get the vertex of the current halfedge
             VertexCIter v = h_twin->vertex(); // vertex is 'source' of the half edge.
             sumx += weight * v->position.x ;
             sumy += weight * v->position.y ;
             sumz += weight * v->position.z ;
             h = h_twin->next(); // move to the next outgoing halfedge of the vertex.
          } while( h != vIter->halfedge() ); // keep going until we're back at the beginning

          // append the location values for the new vertex
          calculatedPosition.x = sumx ;
          calculatedPosition.y = sumy ;
          calculatedPosition.z = sumz ;

          // assign calculated position to calculated position
          vIter->newPosition = calculatedPosition ;
       }

       // compute edge vertices positions
       for (EdgeIter eIter = mesh.edgesBegin() ; eIter != mesh.edgesEnd() ; eIter++) {

          // used to store edge point location
          Vector3D newVertexPosition ;

          // find the directly connected vertices
          VertexCIter v1 = eIter->halfedge()->vertex() ;
          VertexCIter v2 = eIter->halfedge()->twin()->vertex() ;

          // find the opposite connected vertices
          VertexCIter v3 = eIter->halfedge()->next()->next()->vertex() ;
          VertexCIter v4 = eIter->halfedge()->next()->next()->vertex() ;

          // compute location for the edge point on
          // the current edge
          // 3/8 (A+B) + 1/8(C+D)
          sumx = 3/8 * (v1->position.x + v2->position.y) + 1/8 * (v3->position.x + v4->position.x) ;
          sumy = 3/8 * (v1->position.x + v2->position.y) + 1/8 * (v3->position.x + v4->position.x) ;
          sumz = 3/8 * (v1->position.x + v2->position.y) + 1/8 * (v3->position.x + v4->position.x) ;

          // assign values to edge position
          newVertexPosition.x = sumx ;
          newVertexPosition.y = sumy ;
          newVertexPosition.z = sumz ;

          eIter->newPosition = newVertexPosition ;
       }


       // iterate over old edges and split their edges
       EdgeIter e = mesh.edgesBegin();
       while(e != mesh.edgesEnd())
       {
          EdgeIter nextEdge = e;
          nextEdge++;

          if (!e->isNew) {
             VertexIter v = mesh.splitEdge(e) ;
             v->isNew = true ;
             cout << "Created vertex id: " << elementAddress(v) << " degree: " << v->degree() << endl  ;
          }

          e = nextEdge;
       }

       // iterate over the newly created edges
       // flipping it if it connects and old
       // vertex to a new vertex
       EdgeIter f = mesh.edgesBegin();
       while(f != mesh.edgesEnd())
       {
          EdgeIter nextEdge = f;
          nextEdge++;

          // check if an edge connects a new edge to an old one
          // and flip it in this case

          // find the connected vertices
          VertexCIter v1 = f->halfedge()->vertex() ;
          VertexCIter v2 = f->halfedge()->twin()->vertex() ;

          if( (v1->isNew && !v2->isNew) ||
              (!v1->isNew && v2->isNew))
             mesh.flipEdge(f) ;

          f = nextEdge;
       }

       // copy the new vertex positions into the usual
       // vertex positions
       for (VertexIter vIter = mesh.verticesBegin() ; vIter != mesh.verticesEnd(); vIter++) {

          // make sure to only set the position
          // values for the older vertices
          if (!vIter->isNew) {
             vIter->position = vIter->newPosition ;
          }
       }
    }

    // Given an edge, the constructor for EdgeRecord finds the
    // optimal point associated with the edge's current quadric,
    // and assigns this edge a cost based on how much quadric
    // error is observed at this optimal point.
    EdgeRecord::EdgeRecord( EdgeIter& _edge ) : edge( _edge )
    {
       // TODO Compute the combined quadric from the edge endpoints.
       Matrix4x4 e1quadric = _edge->getVertex()->quadric ;
       Matrix4x4 e2quadric =  _edge->getHalfedge()->twin()->getVertex()->quadric ;
       Matrix4x4 combinedQuadric = e1quadric + e2quadric ;

       // TODO Build the 3x3 linear system whose solution minimizes
       // the quadric error associated with these two endpoints
       Matrix3x3 extractedVals ;
       extractedVals(0,0) = combinedQuadric(0,0);
       extractedVals(0,1) = combinedQuadric(0,1);
       extractedVals(0,2) = combinedQuadric(0,2);
       extractedVals(1,0) = combinedQuadric(1,0);
       extractedVals(1,1) = combinedQuadric(1,1);
       extractedVals(1,2) = combinedQuadric(1,2);
       extractedVals(2,0) = combinedQuadric(2,0);
       extractedVals(2,1) = combinedQuadric(2,1);
       extractedVals(2,2) = combinedQuadric(2,2);

       // get the extracted values, get -ve values
       Vector3D extractedVector (combinedQuadric(3,0) * -1,
                               combinedQuadric(3,1) * -1,
                               combinedQuadric(3,2) * -1) ;

       Vector3D linearSystemSol = extractedVals.inv() * extractedVector ;

       // TODO Use this system to solve for the optimal position, and
       // TODO store it in EdgeRecord::optimalPoint.
       _edge->record.optimalPoint = linearSystemSol ;

       // TODO Also store the cost associated with collapsing this edge
       // TODO in EdgeRecord::Cost.
       // _edge->record.score = extractedVals.T() * linearSystemSol.norm() * extractedVals ;

    }

    void MeshResampler::downsample( HalfedgeMesh& mesh )
    {
       // TODO Compute initial quadrics for each face by simply writing the plane
       // equation for the face in homogeneous coordinates.  These quadrics should
       // be stored in Face::quadric


       // TODO Compute an initial quadric for each vertex as the sum of the quadrics
       // associated with the incident faces, storing it in Vertex::quadric


       // TODO Build a priority queue of edges according to their quadric error cost,
       // TODO i.e., by building an EdgeRecord for each edge and sticking it in the queue.


       // TODO Until we reach the target edge budget, collapse the best edge.  Remember
       // TODO to remove from the queue any edge that touches the collapsing edge BEFORE
       // TODO it gets collapsed, and add back into the queue any edge touching the collapsed
       // TODO vertex AFTER it's been collapsed.  Also remember to assign a quadric to the
       // TODO collapsed vertex, and to pop the collapsed edge off the top of the queue.
    }

    void Vertex::computeCentroid( void )
    {
       // TODO Compute the average position of all neighbors of this vertex, and
       // TODO store it in Vertex::centroid.  This value will be used for resampling.
       Vertex* vertex = Vertex::getVertex() ;
       float sumx, sumy, sumz = 0 ;
       int noOfVerticeNeighbors = 0;
       HalfedgeIter h = vertex->halfedge() ;
       do
       {
          VertexCIter v = h->vertex();
          sumx += v->position.x ;
          sumy += v->position.y ;
          sumz += position.z ;
          h = h->next();
          noOfVerticeNeighbors ++ ;
       }
       while( h != vertex->halfedge() );
       Vector3D averagePositions ( (sumx / noOfVerticeNeighbors),
                                   (sumy / noOfVerticeNeighbors),
                                   (sumz / noOfVerticeNeighbors)) ;
       Vertex::centroid = averagePositions ;

    }

    Vector3D Vertex::normal( void ) const
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
       // TODO Compute and return the area-weighted unit normal.
       return Vector3D();
    }

    void MeshResampler::resample( HalfedgeMesh& mesh )
    {
       // TODO Compute the mean edge length.


       // TODO Repeat the four main steps for 5 or 6 iterations


       // TODO Split edges much longer than the target length (being careful about how the loop is written!)


       // TODO Collapse edges much shorter than the target length.  Here we need to be EXTRA careful about
       // TODO advancing the loop, because many edges may have been destroyed by a collapse (which ones?)

       //
       // TODO Now flip each edge if it improves vertex degree


       // TODO Finally, apply some tangential smoothing to the vertex positions
    }
}
