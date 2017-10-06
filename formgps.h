#ifndef FORMGPS_H
#define FORMGPS_H

#include <QMainWindow>
#include <QScopedPointer>
#include <QtOpenGL>

#include "vec2.h"
#include "vec3.h"
#include "vec4.h"
#include "cflag.h"
#include "cmodulecomm.h"
#include "cperimeter.h"
#include "ccamera.h"


namespace Ui {
class FormGPS;
}

//forward declare classes referred to below, to break circular
//references in the code
//class CCamera;
class CWorldGrid;
class CNMEA;
class CSection;
class CABLine;
class CContour;
class CVehicle;
class CPerimeter;
class CBoundary;

//master Manual and Auto, 3 states possible
enum class btnStates {Off,Auto,On};

//section button states
enum class manBtn { Off, Auto, On };


class FormGPS : public QMainWindow
{
    Q_OBJECT
public:
    const int MAXSECTIONS = 9;
    double testDouble = 0;
    bool testBool = false;
    int testInt = 0;
    //current directory of field;
    QString currentFieldDirectory = "";
    QString workingDirectory = "";
    QString vehiclefileName = "";

    //colors for sections and field background
    uchar redSections, grnSections, bluSections;
    uchar redField, grnField, bluField;

    //polygon mode for section drawing
    bool isDrawPolygons = false;

    //for animated submenu
    bool isMenuHid = true;

    //flag for free drive window to control autosteer
    bool isInFreeDriveMode = false;

    //Flag stuff
    uchar flagColor = 0;
    bool leftMouseDownOnOpenGL = false; //mousedown event in opengl window
    int flagNumberPicked = 0;

    //Is it in 2D or 3D, metric, or imperial, display lightbar, display grid
    bool isIn3D = true, isMetric = true, isLightbarOn = true, isGridOn, isSideGuideLines = true;
    bool isPureOn = true;

    //bool for whether or not a job is active
    bool isJobStarted = false, isAreaOnRight = true, isAutoSteerBtnOn = false;

    //master Manual and Auto, 3 states possible
    btnStates manualBtnState = btnStates::Off;
    btnStates autoBtnState = btnStates::Off;

    //if we are saving a file
    bool isSavingFile = false, isLogNMEA = false;

    //Zoom variables
    double gridZoom;
    double zoomValue = 15;
    double triangleResolution = 1.0;
    double previousZoom = 25;

    // Storage For Our Tractor, implement, background etc Textures
    //Texture particleTexture;
    GLuint texture[3];

    //create the scene camera
    CCamera camera;

    //create world grid
    QScopedPointer <CWorldGrid *> worldGrid;

    //create instance of a stopwatch for timing of frames and NMEA hz determination
    //readonly Stopwatch swFrame = new Stopwatch();

    //Time to do fix position update and draw routine
    double frameTime = 0;

    //For field saving in background
    int saveCounter = 1;

    //used to update the screen status bar etc
    int statusUpdateCounter = 1;

    //Parsing object of NMEA sentences
    QScopedPointer<CNMEA *> pn;

    //create an array of sections, so far only 8 section + 1 fullWidth Section
    //CSection[] section = new CSection[MAXSECTIONS];
    QScopedArrayPointer<CSection> section; //not sure this will work.

    //ABLine Instance
    QScopedPointer<CABLine *> ABLine;

    //Contour mode Instance
    QScopedPointer<CContour *> ct;

    //a brand new vehicle
    QScopedPointer<CVehicle *> vehicle;

    //module communication object
    CModuleComm mc;

    //perimeter object for area calc
    CPerimeter periArea;

    //boundary instance
    //CBoundary boundary;

    /*************************
     *  Position.designer.cs *
     *************************/
    double toLatitude;
    double toLongitude;

    //very first fix to setup grid etc
    bool isFirstFixPositionSet = false, isGPSPositionInitialized = false;

    // autosteer variables for sending serial
    short int guidanceLineDistanceOff, guidanceLineSteerAngle;

    //how many fix updates per sec
    int fixUpdateHz = 5;
    double fixUpdateTime = 0.2;

    //for heading or Atan2 as camera
    bool isAtanCam = true;

    //Current fix positions
    double fixEasting = 0.0;
    double fixNorthing = 3.0;
    double fixZ = 0.0;

    Vec2 fix;

    double fixHeadingSection = 0.0, fixHeadingTank = 0.0;
    Vec2 pivotAxlePos;
    Vec2 toolPos;
    Vec2 tankPos;
    Vec2 hitchPos;
    Vec2 prevFix;

    //headings
    double fixHeading = 0.0, fixHeadingCam = 0.0;

    //storage for the cos and sin of heading
    double cosSectionHeading = 1.0, sinSectionHeading = 0.0;

    //a distance between previous and current fix
    double distance = 0.0, userDistance = 0;

    //how far travelled since last section was added, section points
    double sectionTriggerDistance = 0, sectionTriggerStepDistance = 0;
    Vec2 prevSectionPos;

    //step distances and positions for boundary, 6 meters before next point
    double boundaryTriggerDistance = 6.0;
    Vec2 prevBoundaryPos;


    //are we still getting valid data from GPS, resets to 0 in NMEA RMC block, watchdog
    int recvCounter = 20;

    //Everything is so wonky at the start
    int startCounter = 0;

    //individual points for the flags in a list
    QVector<CFlag> flagPts;

    //tally counters for display
    double totalSquareMeters = 0, totalUserSquareMeters = 0, userSquareMetersAlarm = 0;

    //used to determine NMEA sentence frequency
    int timerPn = 1;
    double et = 0, hzTime = 0;

    double avgSpeed[10];//for average speed
    int ringCounter = 0;

    //IMU
    double rollDistance = 0;
    double roll = 0; //, pitch = 0, angVel = 0;
    double avgRoll = 0; //, avgPitch = 0, avgAngVel = 0;

    int times;
    double avgTiltRoll[30];//for tilt
    int ringCounterTiltRoll = 0;
    //public double[] avgTiltPitch = new double[10];//for pitch
    //public int ringCounterTiltPitch = 0;
    //public double[] avgAngularVelocity = new double[30];//for angular velocity
    //public int ringCounterAngularVelocity = 0;

    int totalFixSteps = 10, currentStepFix = 0;
    Vec3 vHold;
    Vec3 stepFixPts[50];
    double distanceCurrentStepFix = 0, fixStepDist, minFixStepDist = 0;
    bool isFixHolding = false, isFixHoldLoaded = false;

    double rollZero = 0, pitchZero = 0;
    double rollAngle = 0, pitchAngle = 0;

    /************************
     * SaveOpen.Designer.cs *
     ************************/
    //list of the list of patch data individual triangles for field sections
    QVector<QSharedPointer<QVector<Vec2>>> patchSaveList;

    //list of the list of patch data individual triangles for contour tracking
    QVector<QSharedPointer<QVector<Vec4>>> contourSaveList;



    explicit FormGPS(QWidget *parent = 0);
    ~FormGPS();

private:
    Ui::FormGPS *ui;
};

#endif // FORMGPS_H