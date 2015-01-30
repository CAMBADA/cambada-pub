#ifndef _VISUAL_HH_
#define _VISUAL_HH_

#include "SingletonT.hh"
#include "XMLConfig.hh"


class QApplication;

namespace visual {
  
  using namespace gazebo;
  
  class MainWindow;
  
  class VisualApp : public SingletonT<VisualApp> {
    
  public:
    
    void Load( XMLConfigNode *node );
    void Init();
    void Fini();
    
    void SetEnabled(bool enabled);
    void ProcessEvents();
    bool UserQuit();
    
    Body* GetBall(){ return this->ball; };
   
  private:

    VisualApp();
    virtual ~VisualApp();
    
    bool enabled;
    
    QApplication*  QTapp;
    MainWindow*    mw;
    
    std::string ballModelName;
    Body* ball;
    
    //Singleton implementation
    friend class DestroyerT<VisualApp>;
    friend class SingletonT<VisualApp>;
    
  };
  
  
}

#endif /* end of include guard: _VISUAL_HH_ */
