
#define dSINGLE 1

#include <QSize>
#include <QString>

#include <vector>

#include <iostream>
#include <iomanip>

#include <vector>
#include <map>
#include <string>
#include <sstream>

#if defined(signals) && defined(QOBJECTDEFS_H) && \
  !defined(QT_MOC_CPP)
#  undef signals
#  define signals signals
#endif

#include "World.hh"

#include "Simulator.hh"
#include "World.hh"
#include "Field.hh"

#include "Model.hh"
#include "Geom.hh"
#include "Body.hh"
#include "Shape.hh"

#include "SphereShape.hh"
#include "BoxShape.hh"
#include "CylinderShape.hh"

#include "Vector2.hh"
#include "Angle.h"

#include "RenderWidget.hh"
#include "internal.h"
#include "Visual.hh"

#include <ode/ode.h>

#if defined(signals) && defined(QOBJECTDEFS_H) && \
  !defined(QT_MOC_CPP)
#  undef signals
// Restore the macro definition of "signals", as it was
// defined by Qt's <qobjectdefs.h>.
#  define signals protected
#endif

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

using namespace cambada::geom;
using namespace gazebo;
using namespace visual;

RenderWidget::RenderWidget(QWidget* parent) : QGLWidget(parent)
{
  setAutoFillBackground(false);

  QGLFormat format;
  format.setDoubleBuffer(true);
  format.setAlpha(true);

  this->setFormat( format );

  this->field = World::Instance()->GetField();
  
  this->hfL = this->field->fieldLength * 0.5 + 1  ; // Half Field Length + adjust
  this->hfW = this->field->fieldWidth  * 0.5 + 1; // half Field Width  + adjust
  
  mx = my = 0;
  mode = 0;
  printHelp = false;
  
  camsmooth[0] = 0;
  camlooksmooth[0] = 0;
  camsmooth[1] = 0;
  camlooksmooth[1] = 0;
  camsmooth[2] = 0;
  camlooksmooth[1] = 0;
  
  this->viewmode = kTopView;
  this->toggleMoveBall = 0;
  this->selectedModel = NULL;
  this->dragMode    = false;
  this->rotateMode  = false;
  this->speedMode   = false;
  
  this->lastTimer = 0;
  
  setFocusPolicy(Qt::StrongFocus);
}

void RenderWidget::initializeGL()
{
    glEnable(GL_MULTISAMPLE);
    initMotionModel();
}

void RenderWidget::paintGL()
//void RenderWidget::paintEvent(QPaintEvent *event)
{
  makeCurrent();

  //draw scene here
  dsDrawFrame( this->w, this->h );
  HandleView();
  
  std::vector<Model*> models = World::Instance()->GetModels();
  std::vector<Model*>::const_iterator m_iter = models.begin();
  
  const std::map< std::string, Body* > *bodies;
  const std::map<std::string, Geom*> *geoms;
  Model* m     = NULL;
  Geom* geom   = NULL;
  Shape* shape = NULL;
  
  dReal p[3];
  dReal r[12];
  dReal q[4];
  
  for ( ; m_iter != models.end(); m_iter++ ){
    
    m = *m_iter;
    if ( m->GetType().compare("empty") == 0 ) continue;
    
    // Find all Bodies
    bodies = m->GetBodies();
    std::map< std::string, Body* >::const_iterator b_iter = bodies->begin();
    
    for ( ; b_iter != bodies->end(); b_iter++ ){
      geoms = b_iter->second->GetGeoms();
      std::map<std::string, Geom*>::const_iterator g_iter = geoms->begin();
      
      for ( ; g_iter != geoms->end(); g_iter++ ){
        geom = g_iter->second;
        
        Vector3 rgb = geom->GetPigment();
        dsSetColor( rgb.x, rgb.y, rgb.z );
        
        shape= geom->GetShape();
        // Body Pose
        Pose3d pose = geom->GetAbsPose();
        
        p[0] = pose.pos.x;
        p[1] = pose.pos.y;
        p[2] = pose.pos.z;
        
        q[0] = pose.rot.u;
        q[1] = pose.rot.x;
        q[2] = pose.rot.y;
        q[3] = pose.rot.z;
        
        dQtoR( q, r );
        
        switch ( shape->GetType() ){
          case Shape::SPHERE:
            dsDrawSphere( p, r , ((SphereShape*)shape)->GetSize());
            break;
            
          case Shape::CYLINDER:
          {
            //cylinder dimension
            Vector2<double>  cyldim = ((CylinderShape*)shape)->GetSize();
            dsDrawCylinder( p, r, cyldim.y, cyldim.x );
          }
            break;
            
          case Shape::BOX:
          {
            float s[3];
            Vector3 v = ((BoxShape*)shape)->GetSize();
            s[0] = v.x; s[1] = v.y; s[2] = v.z;
            dsDrawBox( p, r, s);
          }
            break;
          
          default:
            break;
        }// end Switch geom
        
      } // end for geom..
      
    } // end for bodies
    
  } // end for models 
  
  //dsSetColor (1, 1, 1);

  DrawField();

}

void RenderWidget::resizeGL(int width, int height)
{   
  this->w = width; this->h = height;
      // setup viewport
  glViewport (0,0,width,height);
  glMatrixMode (GL_PROJECTION);
  //glPushMatrix();
  glLoadIdentity();
  
  if ( this->viewmode == RenderWidget::kTopView ){

    double aux;
    aux = height*(this->hfL/this->hfW);
    if (aux < width ){
      lR = ((width-aux)*this->hfL) / aux;
      wR = 0.0;
    }else{
      aux = width * ( this->hfW / this->hfL );
      lR = 0.0;
      wR = ( height - aux ) * ( this->hfW ) / aux;
    }

    glOrtho (-this->hfL - lR ,this->hfL + lR, -this->hfW - wR, this->hfW + wR, -1, 4);

  }else{
  
    const float vnear = 0.1f;
    const float vfar = 100.0f;
    const float k = 0.8f;      // view scale, 1 = +/- 45 degrees
    if (width >= height) {
      float k2 = float(height)/float(width);
      glFrustum (-vnear*k,vnear*k,-vnear*k*k2,vnear*k*k2,vnear,vfar);
    }
    else {
      float k2 = float(width)/float(height);
      glFrustum (-vnear*k*k2,vnear*k*k2,-vnear*k,vnear*k,vnear,vfar);
    }
  
  } // end if( this->viewmode == RenderWidget::kTopView )
}

void RenderWidget::mousePressEvent(QMouseEvent *event)
{

  //proces mouse events for rotate/move inside 3D scene
  
  if ( event->button() == Qt::LeftButton)  mode |= 1;
  if ( event->button() == Qt::MidButton)   mode |= 2;
  if ( event->button() == Qt::RightButton) mode |= 4;
  
  mx =  this->w - event->x();
  my =  this->h - event->y();
  
  if ( this->viewmode == RenderWidget::kTopView ){
    // screen to field
    Vector3 pos;
    
    fx = pos.x = ((this->hfW + wR) * 2.0 * my / this->h) - (this->hfW + wR);
    fy = pos.y = ((this->hfL + lR) * 2.0 * mx / this->w) - (this->hfL + lR);
    
    Model* m = World::Instance()->GetModelAt( pos );
    
    if ( m != NULL && (m->IsStatic() == false) ){
      this->selectedModel = m;
      
      if ( VisualApp::Instance()->GetBall()->GetParentModel() == m ){
        this->speedMode   = bool(mode & 4);
        this->dragMode    = bool(mode & 1);
        this->rotateMode  = false;
      } else {
        this->dragMode    = bool(mode & 1);
        this->speedMode   = false;
        this->rotateMode  = bool(mode & 4);
      }
    }
    
  } // end ( this->viewmode == RenderWidget::kTopView  )
  
}

void RenderWidget::mouseReleaseEvent(QMouseEvent *event)
{
  //proces mouse events for rotate/move inside 3D scene
  
  if ( event->button() == Qt::LeftButton)  mode &= (~1);
  if ( event->button() == Qt::MidButton)   mode &= (~2);
  if ( event->button() == Qt::RightButton) mode &= (~4);
  
  mx =  this->w - event->x();
  my =  this->h - event->y();
  
  if ( (this->viewmode == RenderWidget::kTopView) ){
    
    if ( this->dragMode ){
    
          // screen to field
      Pose3d p = this->selectedModel->GetAbsPose();
    
      p.pos.x = ((this->hfW + wR) * 2.0 * my / this->h) - (this->hfW + wR);
      p.pos.y = ((this->hfL + lR) * 2.0 * mx / this->w) - (this->hfL + lR);
      World::Instance()->SetModelPose( this->selectedModel,  p );
      this->selectedModel->Restore();
    
      this->dragMode      = false;
      this->selectedModel = NULL;
    }
    
    if ( this->rotateMode ){
    
      Pose3d mpos  = this->selectedModel->GetAbsPose();
      Pose3d dummy = Pose3d( mpos );      
      dummy.pos.x = this->fx;
      dummy.pos.y = this->fy;
      Pose3d rel = dummy - mpos;
      
      double  angle = std::atan2( rel.pos.x, rel.pos.y ) - mpos.rot.GetYaw();
      Vector3 axisrot = mpos.rot.GetAsEuler();
      axisrot.z = -angle;
      
      mpos.rot.SetFromEuler( axisrot );
      World::Instance()->SetModelPose( this->selectedModel,  mpos );
    
      this->rotateMode    = false;
      this->selectedModel = NULL;
    } // end if rotateMode
    
    if ( this->speedMode ){
    
      Pose3d mpos  = VisualApp::Instance()->GetBall()->GetAbsPose();
      mpos.rot.SetToIdentity();
      mpos.pos.z = 0.0;
      
      Pose3d dummy = Pose3d( mpos ); 
      dummy.pos.x = this->fx;
      dummy.pos.y = this->fy;
      dummy.pos.z = 0.0;
      Pose3d rel = dummy - mpos;
      
      // Save current velocity
      Vector3 linVel = VisualApp::Instance()->GetBall()->GetLinearVel();
      
      //VisualApp::Instance()->GetBall()->SetEnabled(true);
      VisualApp::Instance()->GetBall()->SetLinearVel( rel.pos );
      // Save ball model state with the new velocity
      World::Instance()->SaveModelState( this->selectedModel );
      // restore velocity
      VisualApp::Instance()->GetBall()->SetLinearVel( linVel );
      
      this->PrintInfo("Ball velocity saved..");
      
      this->speedMode    = false;
      this->selectedModel = NULL;
    } // end if speedMode
    
    this->persistInfo.clear();
  }
  
}

void RenderWidget::mouseMoveEvent(QMouseEvent *event)
{

  if ( viewmode == RenderWidget::kFreeView )
    dsMotion (mode, mx - (this->w - event->x() ),   my - (this->h - event->y()));
  
  mx =  this->w - event->x();
  my =  this->h - event->y();
  
  if ( (this->viewmode == RenderWidget::kTopView) && (this->dragMode || this->rotateMode || this->speedMode) ){
    
    fx = ((this->hfW + wR) * 2.0 * my / this->h) - (this->hfW + wR);
    fy = ((this->hfL + lR) * 2.0 * mx / this->w) - (this->hfL + lR);
    
    if ( this->dragMode ){

    	fprintf(stderr, "obj_name: %s\n",selectedModel->GetName().c_str());

    	QString info = QString("Respawn at (%1,%2)")
    	    		                            .arg( fx, 3, 'f', 2 )
    	    		                            .arg( fy, 3, 'f', 2);

      emit actionInfo( info );
    }
    
    if ( this->rotateMode ){
      Pose3d mpos  = this->selectedModel->GetAbsPose();
      Pose3d dummy = Pose3d( mpos );      
      dummy.pos.x = this->fx;
      dummy.pos.y = this->fy;
      Pose3d rel = dummy - mpos;
      
      // If it's an obstacle, move it while holding the mouse button
          	if(prefix("obstacle_",selectedModel->GetName().c_str()))
          	{
          		QString info = QString("Moving obstacle to (%1,%2)")
          		                            .arg( fx, 3, 'f', 2 )
          		                            .arg( fy, 3, 'f', 2);

          		// screen to field
          		Pose3d p = this->selectedModel->GetAbsPose();

      			p.pos.x = ((this->hfW + wR) * 2.0 * my / this->h) - (this->hfW + wR);
      			p.pos.y = ((this->hfL + lR) * 2.0 * mx / this->w) - (this->hfL + lR);
      			World::Instance()->SetModelPose( this->selectedModel,  p );

      			emit actionInfo( info );
          	}else{
          		QString info = QString("Orientation: %1 (deg)").arg(
          		              - RTOD( std::atan2( rel.pos.x, rel.pos.y ) - mpos.rot.GetYaw() ), 0, 'f', 2 );
          		      emit actionInfo( info );
          	}




    
    }
    
    if ( this->speedMode ){
      Pose3d mpos  = this->selectedModel->GetAbsPose();
      Pose3d dummy = Pose3d( mpos );      
      dummy.pos.x = this->fx;
      dummy.pos.y = this->fy;
      Pose3d rel = dummy - mpos;

      QString info = QString("Speed: %1 (m/s)").arg( rel.pos.GetLength() , 0, 'f', 2 );
      emit actionInfo( info );
    }
    
    return;
  }

}

void RenderWidget::keyPressEvent(QKeyEvent* event)
{
    QString info;

  switch( event->key() ){
    case Qt::Key_Question : 
      viewmode = RenderWidget::kFreeView; 
      this->resizeGL(this->w, this->h); 
      info = QString("FreeView selected");
      emit actionInfo( info );
      return;
      
    case Qt::Key_Plus : 
      viewmode = RenderWidget::kBallView; 
      this->resizeGL(this->w, this->h);
      info = QString("BallView selected");
      emit actionInfo( info );
      return;

    case Qt::Key_0 :
      viewmode = RenderWidget::kTopView;
      this->resizeGL(this->w, this->h); 

      info = QString("TopView selected");
      emit actionInfo( info );
      return;
      
    case Qt::Key_1 :
    case Qt::Key_2 :
    case Qt::Key_3 :
    case Qt::Key_4 :
    case Qt::Key_5 :
    case Qt::Key_6 :
      viewmode = RenderWidget::kRobotView;
      this->watchThisRobot = ( event->key() - Qt::Key_0  );
      this->resizeGL(this->w, this->h);

      info = QString("View of robot %1 selected").arg(this->watchThisRobot);
      emit actionInfo( info );

      return;
      
    case Qt::Key_W:    
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      if( this->toggleMoveBall == 0 )
          ball->SetForce( Vector3(1,0,0) );
      else if( this->toggleMoveBall == 1 )
      {
          Pose3d po = ball->GetAbsPose();
          po.pos += Vector3(0.20,0.0,0.0);
          ball->SetAbsPose(po);
          ball->SetLinearVel( Vector3(0,0,0) );
          ball->SetAngularVel( Vector3(0,0,0) );
      }

      return;
    } // end switch Qt::Key_W
    
    case Qt::Key_S:
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      if( this->toggleMoveBall == 0 )
          ball->SetForce( Vector3(-1,0,0) );
      else if( this->toggleMoveBall == 1 )
      {
          Pose3d po = ball->GetAbsPose();
          po.pos += Vector3(-0.20,0.0,0.0);
          ball->SetAbsPose(po);
          ball->SetLinearVel( Vector3(0,0,0) );
          ball->SetAngularVel( Vector3(0,0,0) );
      }

      return;
    } // end switch Qt::Key_S
    
    case Qt::Key_D:
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      if( this->toggleMoveBall == 0 )
          ball->SetForce( Vector3(0,-1,0) );
      else if( this->toggleMoveBall == 1 )
      {
          Pose3d po = ball->GetAbsPose();
          po.pos += Vector3(0.0,-0.20,0.0);
          ball->SetAbsPose(po);
          ball->SetLinearVel( Vector3(0,0,0) );
          ball->SetAngularVel( Vector3(0,0,0) );
      }

      return;
    } // end switch Qt::Key_D
    
    case Qt::Key_A:
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      if( this->toggleMoveBall == 0 )
          ball->SetForce( Vector3(0,1,0) );
      else if( this->toggleMoveBall == 1 )
      {
          Pose3d po = ball->GetAbsPose();
          po.pos += Vector3(0.0,0.20,0.0);
          ball->SetAbsPose(po);
          ball->SetLinearVel( Vector3(0,0,0) );
          ball->SetAngularVel( Vector3(0,0,0) );
      }

      return;
    } // end switch Qt::Key_A
    
    case Qt::Key_X:
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      ball->SetLinearVel( Vector3(0,0,0) );
      ball->SetAngularVel( Vector3(0,0,0) );

      return;
    } // end switch Qt::Key_X
    
    case Qt::Key_R:
    {
     
      Body* ball = VisualApp::Instance()->GetBall();
      ball->SetEnabled(true);
      ball->SetLinearVel( Vector3(0,0,0) );
      ball->SetAngularVel( Vector3(0,0,0) );

      Pose3d posi;
      posi.pos.z = 0.11;
      ball->SetAbsPose(posi);

      return;
    } // end switch Qt::Key_R
    
    case Qt::Key_B:
    {
      this->toggleMoveBall = 1 - this->toggleMoveBall;

      return;
    } // end switch Qt::Key_B
    
    case Qt::Key_H:
    {
      this->printHelp = not this->printHelp;
      return;
    }

  } 
  
}


void RenderWidget::HandleView(){
  
  float camspeedpos = 0.2;
  float camspeedlook = 0.2;
  static float pos[3];
  static float look[4];

  switch ( this->viewmode ){
  
  case RenderWidget::kFreeView: return; // Free 
  case RenderWidget::kRobotView:
  {
        dReal dpos[3];
        dReal dlook[4];
        
        std::stringstream ss;
        ss << "robbie_" << this->watchThisRobot;
        Model* m = World::Instance()->GetModelByName( ss.str() );
        if ( m == NULL ){
            // default to ball
            m = VisualApp::Instance()->GetBall()->GetParentModel() ;
        }

        Body* body = m->GetCanonicalBody();
        Pose3d bpose = body->GetAbsPose();
        
        double rotation = bpose.rot.GetYaw();

        dpos[0] = bpose.pos.x + (0.5) * std::sin(rotation);
        dpos[1] = bpose.pos.y - (0.5) * std::cos(rotation);
        dpos[2] = bpose.pos.z;

        dlook[0] = bpose.rot.u;
        dlook[1] = bpose.rot.x;
        dlook[2] = bpose.rot.y;
        dlook[3] = bpose.rot.z;
        
        pos[0] = (float) dpos[0];
        pos[1] = (float) dpos[1];
        pos[2] = (float) dpos[2] + 0.4;
        
        cambada::geom::Angle na = cambada::geom::Angle::deg_angle(90.0) + cambada::geom::Angle::rad_angle(rotation);

        look[0] = na.get_deg_180();
        look[1] = -25;
        look[2] = 0.0;
        look[3] = 0.0;

        //look[1] = -25;
        camspeedpos = 0.5;
        camspeedlook = 0.5;
        break;
  }        
  
  case RenderWidget::kBallView:
  { 
    // Ball position
    Body* ball = VisualApp::Instance()->GetBall();
    if ( ball != NULL ){
      dReal dpos[3];
      dReal dlook[4];
      Pose3d bpose = ball->GetAbsPose();
      
      dpos[0] = bpose.pos.x;
      dpos[1] = bpose.pos.y;
      dpos[2] = bpose.pos.z;
      
      dlook[0] = bpose.rot.u;
      dlook[1] = bpose.rot.x;
      dlook[2] = bpose.rot.y;
      dlook[3] = bpose.rot.z;

      pos[0] = (float) dpos[0];
      pos[1] = (float) dpos[1];
      pos[2] = (float) +4.4;
      look[0] = 180;
      look[1] = -90;
      look[2] = 180;
      look[3] = 0;

      camspeedpos = 0.05;
      camspeedlook = 0.05;
      break;
    }
  }
  
  case RenderWidget::kTopView:  
    pos[0] = 0; pos[1] = 0; 
    pos[2] = 3;//10* World::Instance()->GetField()->fieldLength / 12.0;
    look[0] = 180.0; look[1] = -90.0; look[2] = 180.0; look[3] = 0.0;
    
          camspeedpos = 1.0;
      camspeedlook = 1.0;
    break;  
  }
      
  for (int i = 0; i < 3; i++){
    camsmooth[i] = camsmooth[i] * (1 - camspeedpos) + pos[i] * camspeedpos;
    camlooksmooth[i] = camlooksmooth[i] * (1 - camspeedlook) + look[i] * camspeedlook;      
  }
  
  dsSetViewpoint (camsmooth, camlooksmooth);
}


void RenderWidget::DrawField(){

  float lh = 0.03;
  float a[3];
  float b[3];
  
    // not quite on the ground
  a[2] = b[2] = lh;
  
    // should this been done here ???
  if ( (this->dragMode == true) && (this->selectedModel) ){

    Pose3d mpos = this->selectedModel->GetAbsPose();
    a[0] = mpos.pos.x; a[1] = mpos.pos.y;
    b[0] = this->fx; b[1] = this->fy; 
    
    dsSetColor (1, 0, 0);
    dsDrawLine(a, b);
  }
  
    // should this been done here ???
  if ( (this->rotateMode == true) && (this->selectedModel) ){
    Pose3d mpos  = this->selectedModel->GetAbsPose();
    a[0] = mpos.pos.x; a[1] = mpos.pos.y; // Model center
    b[0] = this->fx;
    b[1] = this->fy;    
    dsSetColor (1, 1, 0);
    dsDrawLine(a, b);
  }
  
      // should this been done here ???
  if ( (this->speedMode == true) && (this->selectedModel) ){
    Pose3d mpos  = this->selectedModel->GetAbsPose();
    a[0] = mpos.pos.x; a[1] = mpos.pos.y; // Model center
    b[0] = this->fx;
    b[1] = this->fy;    
    dsSetColor (0, 1, 1);
    dsDrawLine(a, b);
  }
  
  // White lines
  dsSetColor (1, 1, 1);

  std::vector<csim::LineSegment>::iterator iter = this->field->GetSegments()->begin();
  

  
  for ( ; iter != this->field->GetSegments()->end(); ++iter )
  {
    a[0] = (*iter).pointA.x; a[1] = (*iter).pointA.y;
    b[0] = (*iter).pointB.x; b[1] = (*iter).pointB.y;
    
    dsDrawLine(a, b);
  }
  

  
  return;
}


void RenderWidget::PrintInfo(const std::string info){
  this->infoStr = info;
  this->lastTimer = this->startTimer(5000);

}

void RenderWidget::timerEvent(QTimerEvent *event){

  if ( this->lastTimer == event->timerId() )
    this->infoStr = std::string();
  
  this->killTimer( event->timerId() );
}

void RenderWidget::drawInstructions(QPainter *painter, const QString& text)
 {
     QFontMetrics metrics = QFontMetrics(font());
     int border = qMax(4, metrics.leading());

     QRect rect = metrics.boundingRect(0, 0, width() - 2*border, int(height()*0.125),
                                       Qt::AlignLeft | Qt::TextWordWrap, text);

     painter->setRenderHint(QPainter::TextAntialiasing);
     painter->fillRect(QRect(0, 0, width(), rect.height() + 2*border),
                      QColor(0, 0, 0, 100));
     painter->setPen(Qt::white);
     painter->fillRect(QRect(0, 0, width(), rect.height() + 2*border),
                       QColor(0, 0, 0, 100));
     painter->drawText(4, border,
                       rect.width(), rect.height(),
                       Qt::AlignLeft | Qt::TextWordWrap, text);
 }

bool RenderWidget::prefix(const char *pre, const char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}
