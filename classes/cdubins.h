#ifndef CDUBINS_H
#define CDUBINS_H

#include <QVector>
#include <QtMath>
#include <QList>


#include "vec2.h"
#include "vec3.h"
#include "glm.h"

using namespace std;
enum PathType { RSR, LSL, RSL, LSR, RLR, LRL };



class CDubins
{
public:
    CDubins();
    //To keep track of the different paths when debugging

    //How far we are driving each update, the accuracy will improve if we lower the driveDistance
    const double driveDistance = 0.1;

    //The radius the car can turn 360 degrees with
    static double turningRadius;

    //takes 2 points and headings to create a path - returns list of vec3 points and headings
    QVector<Vec3> GenerateDubins(Vec3 _start, Vec3 _goal);
    //takes 2 points and headings to create a path - returns list of vec3 points and headings
    QVector<Vec3> GenerateDubins(Vec3 _start, Vec3 _goal, CGeoFence fence);


private:
    //Position, Heading is in radians
    Vec2 startPos, goalPos;
    double startHeading, goalHeading;
    //The 4 different circles we have that sits to the left/right of the start/goal
    Vec2 startLeftCircle, startRightCircle, goalLeftCircle, goalRightCircle;
    QVector<OneDubinsPath> pathDataList = new QVector<OneDubinsPath>();
    QVector<Vec3> dubinsShortestPathList = new QVector<Vec3>();
    QVector<OneDubinsPath> GetAllDubinsPaths();
    void PositionLeftRightCircles();
    void CalculateDubinsPathsLengths();
    void Get_RSR_Length();
    void Get_LSL_Length();
    void Get_RSL_Length();
    void Get_LSR_Length();
    void Get_RLR_Length();
    void Get_LRL_Length();
    void GeneratePathCoordinates();
    void GetTotalPath(OneDubinsPath pathData);

};


class OneDubinsPath
{
public:
    OneDubinsPath(double length1, double length2, double length3, const Vec2 &tangent1, const Vec2 &tangent2, PathType pathType);
    void SetIfTurningRight(bool segment1TurningRight, bool segment2TurningRight, bool segment3TurningRight);
    //Tthe total length of this path
    double totalLength;

    //Need the individual path lengths for debugging and to find the final path
    double length1, length2, length3;

    //The 2 tangent points we need to connect the lines and curves
    Vec2 tangent1, tangent2;

    //The type, such as RSL
    PathType pathType;

    //The coordinates of the final path
    QVector<Vec2> pathCoordinates;

    //Are we turning or driving straight in segment 2?
    bool segment2Turning;

    //Are we turning right in the particular segment?
    bool segment1TurningRight, segment2TurningRight, segment3TurningRight;
};

class DubinsMath
{
public:
    DubinsMath();
    //Calculate center positions of the Right circle
    static Vec2 GetRightCircleCenterPos(Vec2 circlePos, double heading);
    //Calculate center positions of the Left circle
    static Vec2 GetLeftCircleCenterPos(Vec2 circlePos, double heading);
    //Outer tangent (LSL and RSR)
    void LSLorRSR(Vec2 startCircle, Vec2 goalCircle, bool isBottom, Vec2 &startTangent, Vec2 &goalTangent);
    //Inner tangent (RSL and LSR)
    void RSLorLSR(Vec2 startCircle, Vec2 goalCircle, bool isBottom, Vec2 &startTangent, Vec2 &goalTangent);
    //Get the RLR or LRL tangent points
    void GetRLRorLRLTangents(Vec2 startCircle, Vec2 goalCircle, bool isLRL, const Vec2 &startTangent, const Vec2 &goalTangent, const Vec2 &middleCircle);
    //Calculate the length of an circle arc depending on which direction we are driving
    double GetArcLength(Vec2 circleCenterPos, Vec2 startPos, Vec2 goalPos, bool isLeftCircle);
    //Loops through segments of a path and add new coordinates to the final path
    void AddCoordinatesToPath(Vec2 currentPos, double theta, QVector<Vec2>finalPath, int segments, bool isTurning, bool isTurningRight);


};



#endif // CDUBINS_H
