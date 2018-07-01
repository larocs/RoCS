// Use this header to dynamically load and bind v_rep.dll and its functions (call loadVrepLibrary and unloadVrepLibrary)

#if !defined(V_REPLIB_INCLUDED_)
#define V_REPLIB_INCLUDED_

#include "v_repConst.h"
#include "v_repTypes.h"

#ifdef QT_FRAMEWORK
																																																												#include <QLibrary>
    #define LIBRARY QLibrary*
    #define __cdecl
    #define FARPROC void*
#else
#ifdef _WIN32
																																																												#include <Windows.h>
        #define LIBRARY HINSTANCE
#elif defined (__linux) || defined (__APPLE__)
#define __cdecl
#define FARPROC void*
#define LIBRARY void*
#endif
#endif // QT_FRAMEWORK

int getVrepProcAddresses(LIBRARY lib);

LIBRARY loadVrepLibrary(const char *pathAndFilename);

void unloadVrepLibrary(LIBRARY lib);

FARPROC _getProcAddress(LIBRARY lib, const char *funcName);

typedef simInt (__cdecl *ptrSimRunSimulator)(const simChar *applicationName, simInt options, simVoid(*initCallBack)(), simVoid(*loopCallBack)(), simVoid(*deinitCallBack)());

typedef simChar *(__cdecl *ptrSimGetSimulatorMessage)(simInt *messageID, simInt *auxiliaryData, simInt *returnedDataSize);

typedef simVoid *(__cdecl *ptrSimGetMainWindow)(simInt type);

typedef simChar *(__cdecl *ptrSimGetLastError)();

typedef simInt (__cdecl *ptrSimLoadModule)(const simChar *filenameAndPath, const simChar *pluginName);

typedef simInt (__cdecl *ptrSimUnloadModule)(simInt pluginhandle);

typedef simVoid *(__cdecl *ptrSimSendModuleMessage)(simInt message, simInt *auxiliaryData, simVoid *customData, simInt *replyData);

typedef simInt (__cdecl *ptrSimSetBooleanParameter)(simInt parameter, simBool boolState);

typedef simInt (__cdecl *ptrSimGetBooleanParameter)(simInt parameter);

typedef simInt (__cdecl *ptrSimSetBoolParameter)(simInt parameter, simBool boolState);

typedef simInt (__cdecl *ptrSimGetBoolParameter)(simInt parameter);

typedef simInt (__cdecl *ptrSimSetIntegerParameter)(simInt parameter, simInt intState);

typedef simInt (__cdecl *ptrSimGetIntegerParameter)(simInt parameter, simInt *intState);

typedef simInt (__cdecl *ptrSimSetInt32Parameter)(simInt parameter, simInt intState);

typedef simInt (__cdecl *ptrSimGetInt32Parameter)(simInt parameter, simInt *intState);

typedef simInt (__cdecl *ptrSimGetUInt64Parameter)(simInt parameter, simUInt64 *intState);

typedef simInt (__cdecl *ptrSimSetFloatingParameter)(simInt parameter, simFloat floatState);

typedef simInt (__cdecl *ptrSimGetFloatingParameter)(simInt parameter, simFloat *floatState);

typedef simInt (__cdecl *ptrSimSetFloatParameter)(simInt parameter, simFloat floatState);

typedef simInt (__cdecl *ptrSimGetFloatParameter)(simInt parameter, simFloat *floatState);

typedef simInt (__cdecl *ptrSimSetStringParameter)(simInt parameter, const simChar *stringState);

typedef simChar *(__cdecl *ptrSimGetStringParameter)(simInt parameter);

typedef simInt (__cdecl *ptrSimGetObjectHandle)(const simChar *objectName);

typedef simInt (__cdecl *ptrSimRemoveObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimRemoveModel)(simInt objectHandle);

typedef simChar *(__cdecl *ptrSimGetObjectName)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetObjects)(simInt index, simInt objectType);

typedef simInt (__cdecl *ptrSimSetObjectName)(simInt objectHandle, const simChar *objectName);

typedef simInt (__cdecl *ptrSimGetCollectionHandle)(const simChar *collectionName);

typedef simInt (__cdecl *ptrSimRemoveCollection)(simInt collectionHandle);

typedef simInt (__cdecl *ptrSimEmptyCollection)(simInt collectionHandle);

typedef simChar *(__cdecl *ptrSimGetCollectionName)(simInt collectionHandle);

typedef simInt (__cdecl *ptrSimSetCollectionName)(simInt collectionHandle, const simChar *collectionName);

typedef simInt (__cdecl *ptrSimGetObjectMatrix)(simInt objectHandle, simInt relativeToObjectHandle, simFloat *matrix);

typedef simInt (__cdecl *ptrSimSetObjectMatrix)(simInt objectHandle, simInt relativeToObjectHandle, const simFloat *matrix);

typedef simInt (__cdecl *ptrSimGetObjectPosition)(simInt objectHandle, simInt relativeToObjectHandle, simFloat *position);

typedef simInt (__cdecl *ptrSimSetObjectPosition)(simInt objectHandle, simInt relativeToObjectHandle, const simFloat *position);

typedef simInt (__cdecl *ptrSimGetObjectOrientation)(simInt objectHandle, simInt relativeToObjectHandle, simFloat *eulerAngles);

typedef simInt (__cdecl *ptrSimSetObjectOrientation)(simInt objectHandle, simInt relativeToObjectHandle, const simFloat *eulerAngles);

typedef simInt (__cdecl *ptrSimGetJointPosition)(simInt objectHandle, simFloat *position);

typedef simInt (__cdecl *ptrSimSetJointPosition)(simInt objectHandle, simFloat position);

typedef simInt (__cdecl *ptrSimSetJointTargetPosition)(simInt objectHandle, simFloat targetPosition);

typedef simInt (__cdecl *ptrSimGetJointTargetPosition)(simInt objectHandle, simFloat *targetPosition);

typedef simInt (__cdecl *ptrSimSetJointForce)(simInt objectHandle, simFloat forceOrTorque);

typedef simInt (__cdecl *ptrSimGetPathPosition)(simInt objectHandle, simFloat *position);

typedef simInt (__cdecl *ptrSimSetPathPosition)(simInt objectHandle, simFloat position);

typedef simInt (__cdecl *ptrSimGetPathLength)(simInt objectHandle, simFloat *length);

typedef simInt (__cdecl *ptrSimGetJointMatrix)(simInt objectHandle, simFloat *matrix);

typedef simInt (__cdecl *ptrSimSetSphericalJointMatrix)(simInt objectHandle, const simFloat *matrix);

typedef simInt (__cdecl *ptrSimGetJointInterval)(simInt objectHandle, simBool *cyclic, simFloat *interval);

typedef simInt (__cdecl *ptrSimSetJointInterval)(simInt objectHandle, simBool cyclic, const simFloat *interval);

typedef simInt (__cdecl *ptrSimGetObjectParent)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetObjectChild)(simInt objectHandle, simInt index);

typedef simInt (__cdecl *ptrSimSetObjectParent)(simInt objectHandle, simInt parentObjectHandle, simBool keepInPlace);

typedef simInt (__cdecl *ptrSimGetObjectType)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetJointType)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimBuildIdentityMatrix)(simFloat *matrix);

typedef simInt (__cdecl *ptrSimCopyMatrix)(const simFloat *matrixIn, simFloat *matrixOut);

typedef simInt (__cdecl *ptrSimBuildMatrix)(const simFloat *position, const simFloat *eulerAngles, simFloat *matrix);

typedef simInt (__cdecl *ptrSimGetEulerAnglesFromMatrix)(const simFloat *matrix, simFloat *eulerAngles);

typedef simInt (__cdecl *ptrSimInvertMatrix)(simFloat *matrix);

typedef simInt (__cdecl *ptrSimMultiplyMatrices)(const simFloat *matrixIn1, const simFloat *matrixIn2, simFloat *matrixOut);

typedef simInt (__cdecl *ptrSimInterpolateMatrices)(const simFloat *matrixIn1, const simFloat *matrixIn2, simFloat interpolFactor, simFloat *matrixOut);

typedef simInt (__cdecl *ptrSimTransformVector)(const simFloat *matrix, simFloat *vect);

typedef simInt (__cdecl *ptrSimReservedCommand)(simInt v, simInt w);

typedef simFloat (__cdecl *ptrSimGetSimulationTime)();

typedef simInt (__cdecl *ptrSimGetSimulationState)();

typedef simFloat (__cdecl *ptrSimGetSystemTime)();

typedef simInt (__cdecl *ptrSimGetSystemTimeInMilliseconds)();

typedef simUInt (__cdecl *ptrSimGetSystemTimeInMs)(simInt previousTime);

typedef simInt (__cdecl *ptrSimLoadScene)(const simChar *filename);

typedef simInt (__cdecl *ptrSimCloseScene)();

typedef simInt (__cdecl *ptrSimSaveScene)(const simChar *filename);

typedef simInt (__cdecl *ptrSimLoadModel)(const simChar *filename);

typedef simInt (__cdecl *ptrSimSaveModel)(int baseOfModelHandle, const simChar *filename);

typedef simInt (__cdecl *ptrSimAddStatusbarMessage)(const simChar *message);

typedef simInt (__cdecl *ptrSimAddModuleMenuEntry)(const simChar *entryLabel, simInt itemCount, simInt *itemHandles);

typedef simInt (__cdecl *ptrSimSetModuleMenuItemState)(simInt itemHandle, simInt state, const simChar *label);

typedef simInt (__cdecl *ptrSimDoesFileExist)(const simChar *filename);

typedef simInt (__cdecl *ptrSimIsObjectInSelection)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimAddObjectToSelection)(simInt what, simInt objectHandle);

typedef simInt (__cdecl *ptrSimRemoveObjectFromSelection)(simInt what, simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetObjectSelectionSize)();

typedef simInt (__cdecl *ptrSimGetObjectLastSelection)();

typedef simInt (__cdecl *ptrSimGetObjectSelection)(simInt *objectHandles);

typedef simInt (__cdecl *ptrSimHandleCollision)(simInt collisionObjectHandle);

typedef simInt (__cdecl *ptrSimReadCollision)(simInt collisionObjectHandle);

typedef simInt (__cdecl *ptrSimHandleDistance)(simInt distanceObjectHandle, simFloat *smallestDistance);

typedef simInt (__cdecl *ptrSimReadDistance)(simInt distanceObjectHandle, simFloat *smallestDistance);

typedef simInt (__cdecl *ptrSimHandleProximitySensor)(simInt sensorHandle, simFloat *detectedPoint, simInt *detectedObjectHandle, simFloat *normalVector);

typedef simInt (__cdecl *ptrSimReadProximitySensor)(simInt sensorHandle, simFloat *detectedPoint, simInt *detectedObjectHandle, simFloat *normalVector);

typedef simInt (__cdecl *ptrSimHandleMill)(simInt millHandle, simFloat *removedSurfaceAndVolume);

typedef simInt (__cdecl *ptrSimHandleIkGroup)(simInt ikGroupHandle);

typedef simInt (__cdecl *ptrSimCheckIkGroup)(simInt ikGroupHandle, simInt jointCnt, const simInt *jointHandles, simFloat *jointValues, const simInt *jointOptions);

typedef simInt (__cdecl *ptrSimHandleDynamics)(simFloat deltaTime);

typedef simInt (__cdecl *ptrSimGetMechanismHandle)(const simChar *mechanismName);

typedef simInt (__cdecl *ptrSimHandleMechanism)(simInt mechanismHandle);

typedef simInt (__cdecl *ptrSimGetScriptHandle)(const simChar *scriptName);

typedef simInt (__cdecl *ptrSimSetScriptText)(simInt scriptHandle, const simChar *scriptText);

typedef const simChar *(__cdecl *ptrSimGetScriptText)(simInt scriptHandle);

typedef simInt (__cdecl *ptrSimGetScriptProperty)(simInt scriptHandle, simInt *scriptProperty, simInt *associatedObjectHandle);

typedef simInt (__cdecl *ptrSimAssociateScriptWithObject)(simInt scriptHandle, simInt associatedObjectHandle);

typedef simInt (__cdecl *ptrSimGetScript)(simInt index);

typedef simInt (__cdecl *ptrSimGetScriptAssociatedWithObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetCustomizationScriptAssociatedWithObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetObjectAssociatedWithScript)(simInt scriptHandle);

typedef simChar *(__cdecl *ptrSimGetScriptName)(simInt scriptHandle);

typedef simInt (__cdecl *ptrSimHandleMainScript)();

typedef simInt (__cdecl *ptrSimResetScript)(simInt scriptHandle);

typedef simInt (__cdecl *ptrSimAddScript)(simInt scriptProperty);

typedef simInt (__cdecl *ptrSimRemoveScript)(simInt scriptHandle);

typedef simInt (__cdecl *ptrSimRefreshDialogs)(simInt refreshDegree);

typedef simInt (__cdecl *ptrSimGetCollisionHandle)(const simChar *collisionObjectName);

typedef simInt (__cdecl *ptrSimGetDistanceHandle)(const simChar *distanceObjectName);

typedef simInt (__cdecl *ptrSimGetIkGroupHandle)(const simChar *ikGroupName);

typedef simInt (__cdecl *ptrSimResetCollision)(simInt collisionObjectHandle);

typedef simInt (__cdecl *ptrSimResetDistance)(simInt distanceObjectHandle);

typedef simInt (__cdecl *ptrSimResetProximitySensor)(simInt sensorHandle);

typedef simInt (__cdecl *ptrSimResetMill)(simInt millHandle);

typedef simInt (__cdecl *ptrSimCheckProximitySensor)(simInt sensorHandle, simInt entityHandle, simFloat *detectedPoint);

typedef simInt (__cdecl *ptrSimCheckProximitySensorEx)(simInt sensorHandle, simInt entityHandle, simInt detectionMode, simFloat detectionThreshold, simFloat maxAngle, simFloat *detectedPoint, simInt *detectedObjectHandle, simFloat *normalVector);

typedef simInt (__cdecl *ptrSimCheckProximitySensorEx2)(simInt sensorHandle, simFloat *vertexPointer, simInt itemType, simInt itemCount, simInt detectionMode, simFloat detectionThreshold, simFloat maxAngle, simFloat *detectedPoint, simFloat *normalVector);

typedef simChar *(__cdecl *ptrSimCreateBuffer)(simInt size);

typedef simInt (__cdecl *ptrSimReleaseBuffer)(simChar *buffer);

typedef simInt (__cdecl *ptrSimCheckCollision)(simInt entity1Handle, simInt entity2Handle);

typedef simInt (__cdecl *ptrSimCheckCollisionEx)(simInt entity1Handle, simInt entity2Handle, simFloat **intersectionSegments);

typedef simInt (__cdecl *ptrSimCheckDistance)(simInt entity1Handle, simInt entity2Handle, simFloat threshold, simFloat *distanceData);

typedef simChar *(__cdecl *ptrSimGetObjectConfiguration)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetObjectConfiguration)(const simChar *data);

typedef simChar *(__cdecl *ptrSimGetConfigurationTree)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetConfigurationTree)(const simChar *data);

typedef simInt (__cdecl *ptrSimSetSimulationTimeStep)(simFloat timeStep);

typedef simFloat (__cdecl *ptrSimGetSimulationTimeStep)();

typedef simInt (__cdecl *ptrSimGetRealTimeSimulation)();

typedef simInt (__cdecl *ptrSimIsRealTimeSimulationStepNeeded)();

typedef simInt (__cdecl *ptrSimAdjustRealTimeTimer)(simInt instanceIndex, simFloat deltaTime);

typedef simInt (__cdecl *ptrSimGetSimulationPassesPerRenderingPass)();

typedef simInt (__cdecl *ptrSimAdvanceSimulationByOneStep)();

typedef simInt (__cdecl *ptrSimStartSimulation)();

typedef simInt (__cdecl *ptrSimStopSimulation)();

typedef simInt (__cdecl *ptrSimPauseSimulation)();

typedef simVoid *(__cdecl *ptrSimBroadcastMessage)(simInt *auxiliaryData, simVoid *customData, simInt *replyData);

typedef simChar *(__cdecl *ptrSimGetModuleName)(simInt index, simUChar *moduleVersion);

typedef simChar *(__cdecl *ptrSimGetScriptSimulationParameter)(simInt scriptHandle, const simChar *parameterName, simInt *parameterLength);

typedef simInt (__cdecl *ptrSimSetScriptSimulationParameter)(simInt scriptHandle, const simChar *parameterName, const simChar *parameterValue, simInt parameterLength);

typedef simInt (__cdecl *ptrSimFloatingViewAdd)(simFloat posX, simFloat posY, simFloat sizeX, simFloat sizeY, simInt options);

typedef simInt (__cdecl *ptrSimFloatingViewRemove)(simInt floatingViewHandle);

typedef simInt (__cdecl *ptrSimAdjustView)(simInt viewHandleOrIndex, simInt associatedViewableObjectHandle, simInt options, const simChar *viewLabel);

typedef simInt (__cdecl *ptrSimSetLastError)(const simChar *funcName, const simChar *errorMessage);

typedef simInt (__cdecl *ptrSimHandleGraph)(simInt graphHandle, simFloat simulationTime);

typedef simInt (__cdecl *ptrSimResetGraph)(simInt graphHandle);

typedef simInt (__cdecl *ptrSimSetNavigationMode)(simInt navigationMode);

typedef simInt (__cdecl *ptrSimGetNavigationMode)();

typedef simInt (__cdecl *ptrSimSetPage)(simInt index);

typedef simInt (__cdecl *ptrSimGetPage)();

typedef simInt (__cdecl *ptrSimDisplayDialog)(const simChar *titleText, const simChar *mainText, simInt dialogType, const simChar *initialText, const simFloat *titleColors, const simFloat *dialogColors, simInt *elementHandle);

typedef simInt (__cdecl *ptrSimGetDialogResult)(simInt genericDialogHandle);

typedef simChar *(__cdecl *ptrSimGetDialogInput)(simInt genericDialogHandle);

typedef simInt (__cdecl *ptrSimEndDialog)(simInt genericDialogHandle);

typedef simInt (__cdecl *ptrSimRegisterScriptCallbackFunction)(const simChar *funcNameAtPluginName, const simChar *callTips, simVoid(*callBack)(struct SScriptCallBack *cb));

typedef simInt (__cdecl *ptrSimRegisterScriptVariable)(const simChar *varName, const simChar *varValue, simInt stackHandle);

typedef simInt (__cdecl *ptrSimSetJointTargetVelocity)(simInt objectHandle, simFloat targetVelocity);

typedef simInt (__cdecl *ptrSimGetJointTargetVelocity)(simInt objectHandle, simFloat *targetVelocity);

typedef simInt (__cdecl *ptrSimSetPathTargetNominalVelocity)(simInt objectHandle, simFloat targetNominalVelocity);

typedef simChar *(__cdecl *ptrSimGetScriptRawBuffer)(simInt scriptHandle, simInt bufferHandle);

typedef simInt (__cdecl *ptrSimSetScriptRawBuffer)(simInt scriptHandle, const simChar *buffer, simInt bufferSize);

typedef simInt (__cdecl *ptrSimReleaseScriptRawBuffer)(simInt scriptHandle, simInt bufferHandle);

typedef simInt (__cdecl *ptrSimCopyPasteObjects)(simInt *objectHandles, simInt objectCount, simInt options);

typedef simInt (__cdecl *ptrSimScaleSelectedObjects)(simFloat scalingFactor, simBool scalePositionsToo);

typedef simInt (__cdecl *ptrSimScaleObjects)(const simInt *objectHandles, simInt objectCount, simFloat scalingFactor, simBool scalePositionsToo);

typedef simInt (__cdecl *ptrSimDeleteSelectedObjects)();

typedef simInt (__cdecl *ptrSimGetObjectUniqueIdentifier)(simInt objectHandle, simInt *uniqueIdentifier);

typedef simInt (__cdecl *ptrSimGetNameSuffix)(const simChar *name);

typedef simInt (__cdecl *ptrSimSendData)(simInt targetID, simInt dataHeader, const simChar *dataName, const simChar *data, simInt dataLength, simInt antennaHandle, simFloat actionRadius, simFloat emissionAngle1, simFloat emissionAngle2, simFloat persistence);

typedef simChar *(__cdecl *ptrSimReceiveData)(simInt dataHeader, const simChar *dataName, simInt antennaHandle, simInt index, simInt *dataLength, simInt *senderID, simInt *dataHeaderR, simChar **dataNameR);

typedef simInt (__cdecl *ptrSimSetGraphUserData)(simInt graphHandle, const simChar *dataStreamName, simFloat data);

typedef simInt (__cdecl *ptrSimSetNameSuffix)(simInt nameSuffixNumber);

typedef simInt (__cdecl *ptrSimAddDrawingObject)(simInt objectType, simFloat size, simFloat duplicateTolerance, simInt parentObjectHandle, simInt maxItemCount, const simFloat *ambient_diffuse, const simFloat *setToNULL, const simFloat *specular, const simFloat *emission);

typedef simInt (__cdecl *ptrSimRemoveDrawingObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimAddDrawingObjectItem)(simInt objectHandle, const simFloat *itemData);

typedef simInt (__cdecl *ptrSimAddParticleObject)(simInt objectType, simFloat size, simFloat density, const simVoid *params, simFloat lifeTime, simInt maxItemCount, const simFloat *ambient_diffuse, const simFloat *setToNULL, const simFloat *specular, const simFloat *emission);

typedef simInt (__cdecl *ptrSimRemoveParticleObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimAddParticleObjectItem)(simInt objectHandle, const simFloat *itemData);

typedef simFloat (__cdecl *ptrSimGetObjectSizeFactor)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimAnnounceSceneContentChange)();

typedef simInt (__cdecl *ptrSimResetMilling)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimApplyMilling)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetIntegerSignal)(const simChar *signalName, simInt signalValue);

typedef simInt (__cdecl *ptrSimGetIntegerSignal)(const simChar *signalName, simInt *signalValue);

typedef simInt (__cdecl *ptrSimClearIntegerSignal)(const simChar *signalName);

typedef simInt (__cdecl *ptrSimSetFloatSignal)(const simChar *signalName, simFloat signalValue);

typedef simInt (__cdecl *ptrSimGetFloatSignal)(const simChar *signalName, simFloat *signalValue);

typedef simInt (__cdecl *ptrSimClearFloatSignal)(const simChar *signalName);

typedef simInt (__cdecl *ptrSimSetStringSignal)(const simChar *signalName, const simChar *signalValue, simInt stringLength);

typedef simChar *(__cdecl *ptrSimGetStringSignal)(const simChar *signalName, simInt *stringLength);

typedef simInt (__cdecl *ptrSimClearStringSignal)(const simChar *signalName);

typedef simChar *(__cdecl *ptrSimGetSignalName)(simInt signalIndex, simInt signalType);

typedef simInt (__cdecl *ptrSimSetObjectProperty)(simInt objectHandle, simInt prop);

typedef simInt (__cdecl *ptrSimGetObjectProperty)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetObjectSpecialProperty)(simInt objectHandle, simInt prop);

typedef simInt (__cdecl *ptrSimGetObjectSpecialProperty)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetPositionOnPath)(simInt pathHandle, simFloat relativeDistance, simFloat *position);

typedef simInt (__cdecl *ptrSimGetOrientationOnPath)(simInt pathHandle, simFloat relativeDistance, simFloat *eulerAngles);

typedef simInt (__cdecl *ptrSimGetDataOnPath)(simInt pathHandle, simFloat relativeDistance, simInt dataType, simInt *intData, simFloat *floatData);

typedef simInt (__cdecl *ptrSimGetClosestPositionOnPath)(simInt pathHandle, simFloat *absolutePosition, simFloat *pathPosition);

typedef simInt (__cdecl *ptrSimReadForceSensor)(simInt objectHandle, simFloat *forceVector, simFloat *torqueVector);

typedef simInt (__cdecl *ptrSimBreakForceSensor)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimGetShapeVertex)(simInt shapeHandle, simInt groupElementIndex, simInt vertexIndex, simFloat *relativePosition);

typedef simInt (__cdecl *ptrSimGetShapeTriangle)(simInt shapeHandle, simInt groupElementIndex, simInt triangleIndex, simInt *vertexIndices, simFloat *triangleNormals);

typedef simInt (__cdecl *ptrSimSetLightParameters)(simInt objectHandle, simInt state, const simFloat *setToNULL, const simFloat *diffusePart, const simFloat *specularPart);

typedef simInt (__cdecl *ptrSimGetLightParameters)(simInt objectHandle, simFloat *setToNULL, simFloat *diffusePart, simFloat *specularPart);

typedef simInt (__cdecl *ptrSimGetVelocity)(simInt shapeHandle, simFloat *linearVelocity, simFloat *angularVelocity);

typedef simInt (__cdecl *ptrSimGetObjectVelocity)(simInt objectHandle, simFloat *linearVelocity, simFloat *angularVelocity);

typedef simInt (__cdecl *ptrSimAddForceAndTorque)(simInt shapeHandle, const simFloat *force, const simFloat *torque);

typedef simInt (__cdecl *ptrSimAddForce)(simInt shapeHandle, const simFloat *position, const simFloat *force);

typedef simInt (__cdecl *ptrSimSetExplicitHandling)(simInt generalObjectHandle, int explicitFlags);

typedef simInt (__cdecl *ptrSimGetExplicitHandling)(simInt generalObjectHandle);

typedef simInt (__cdecl *ptrSimGetLinkDummy)(simInt dummyHandle);

typedef simInt (__cdecl *ptrSimSetLinkDummy)(simInt dummyHandle, simInt linkedDummyHandle);

typedef simInt (__cdecl *ptrSimSetModelProperty)(simInt objectHandle, simInt modelProperty);

typedef simInt (__cdecl *ptrSimGetModelProperty)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetShapeColor)(simInt shapeHandle, const simChar *colorName, simInt colorComponent, const simFloat *rgbData);

typedef simInt (__cdecl *ptrSimGetShapeColor)(simInt shapeHandle, const simChar *colorName, simInt colorComponent, simFloat *rgbData);

typedef simInt (__cdecl *ptrSimResetDynamicObject)(simInt objectHandle);

typedef simInt (__cdecl *ptrSimSetJointMode)(simInt jointHandle, simInt jointMode, simInt options);

typedef simInt (__cdecl *ptrSimGetJointMode)(simInt jointHandle, simInt *options);

typedef simInt (__cdecl *ptrSimSerialOpen)(const simChar *portString, simInt baudRate, simVoid *reserved1, simVoid *reserved2);

typedef simInt (__cdecl *ptrSimSerialClose)(simInt portHandle);

typedef simInt (__cdecl *ptrSimSerialSend)(simInt portHandle, const simChar *data, simInt dataLength);

typedef simInt (__cdecl *ptrSimSerialRead)(simInt portHandle, simChar *buffer, simInt dataLengthToRead);

typedef simInt (__cdecl *ptrSimSerialCheck)(simInt portHandle);

typedef simInt (__cdecl *ptrSimGetContactInfo)(simInt dynamicPass, simInt objectHandle, simInt index, simInt *objectHandles, simFloat *contactInfo);

typedef simInt (__cdecl *ptrSimSetThreadIsFree)(simBool freeMode);

typedef simInt (__cdecl *ptrSimTubeOpen)(simInt dataHeader, const simChar *dataName, simInt readBufferSize, simBool notUsedButKeepFalse);

typedef simInt (__cdecl *ptrSimTubeClose)(simInt tubeHandle);

typedef simInt (__cdecl *ptrSimTubeWrite)(simInt tubeHandle, const simChar *data, simInt dataLength);

typedef simChar *(__cdecl *ptrSimTubeRead)(simInt tubeHandle, simInt *dataLength);

typedef simInt (__cdecl *ptrSimTubeStatus)(simInt tubeHandle, simInt *readPacketsCount, simInt *writePacketsCount);

typedef simInt (__cdecl *ptrSimAuxiliaryConsoleOpen)(const simChar *title, simInt maxLines, simInt mode, const simInt *position, const simInt *size, const simFloat *textColor, const simFloat *backgroundColor);

typedef simInt (__cdecl *ptrSimAuxiliaryConsoleClose)(simInt consoleHandle);

typedef simInt (__cdecl *ptrSimAuxiliaryConsoleShow)(simInt consoleHandle, simBool showState);

typedef simInt (__cdecl *ptrSimAuxiliaryConsolePrint)(simInt consoleHandle, const simChar *text);

typedef simInt (__cdecl *ptrSimImportShape)(simInt fileformat, const simChar *pathAndFilename, simInt options, simFloat identicalVerticeTolerance, simFloat scalingFactor);

typedef simInt (__cdecl *ptrSimImportMesh)(simInt fileformat, const simChar *pathAndFilename, simInt options, simFloat identicalVerticeTolerance, simFloat scalingFactor, simFloat ***vertices, simInt **verticesSizes, simInt ***indices, simInt **indicesSizes, simFloat ***reserved, simChar ***names);

typedef simInt (__cdecl *ptrSimExportMesh)(simInt fileformat, const simChar *pathAndFilename, simInt options, simFloat scalingFactor, simInt elementCount, simFloat **vertices, const simInt *verticesSizes, simInt **indices, const simInt *indicesSizes, simFloat **reserved, simChar **names);

typedef simInt (__cdecl *ptrSimCreateMeshShape)(simInt options, simFloat shadingAngle, const simFloat *vertices, simInt verticesSize, const simInt *indices, simInt indicesSize, simFloat *reserved);

typedef simInt (__cdecl *ptrSimCreatePureShape)(simInt primitiveType, simInt options, const simFloat *sizes, simFloat mass, const simInt *precision);

typedef simInt (__cdecl *ptrSimCreateHeightfieldShape)(simInt options, simFloat shadingAngle, simInt xPointCount, simInt yPointCount, simFloat xSize, const simFloat *heights);

typedef simInt (__cdecl *ptrSimGetShapeMesh)(simInt shapeHandle, simFloat **vertices, simInt *verticesSize, simInt **indices, simInt *indicesSize, simFloat **normals);

typedef simInt (__cdecl *ptrSimAddBanner)(const simChar *label, simFloat size, simInt options, const simFloat *positionAndEulerAngles, simInt parentObjectHandle, const simFloat *labelColors, const simFloat *backgroundColors);

typedef simInt (__cdecl *ptrSimRemoveBanner)(simInt bannerID);

typedef simInt (__cdecl *ptrSimCreateJoint)(simInt jointType, simInt jointMode, simInt options, const simFloat *sizes, const simFloat *colorA, const simFloat *colorB);

typedef simInt (__cdecl *ptrSimCreateDummy)(simFloat size, const simFloat *color);

typedef simInt (__cdecl *ptrSimCreateForceSensor)(simInt options, const simInt *intParams, const simFloat *floatParams, const simFloat *color);

typedef simInt (__cdecl *ptrSimCreateVisionSensor)(simInt options, const simInt *intParams, const simFloat *floatParams, const simFloat *color);

typedef simInt (__cdecl *ptrSimCreateProximitySensor)(simInt sensorType, simInt subType, simInt options, const simInt *intParams, const simFloat *floatParams, const simFloat *color);

typedef simInt (__cdecl *ptrSimCreatePath)(simInt attributes, const simInt *intParams, const simFloat *floatParams, const simFloat *color);

typedef simInt (__cdecl *ptrSimInsertPathCtrlPoints)(simInt pathHandle, simInt options, simInt startIndex, simInt ptCnt, const simVoid *ptData);

typedef simInt (__cdecl *ptrSimCutPathCtrlPoints)(simInt pathHandle, simInt startIndex, simInt ptCnt);

typedef simInt (__cdecl *ptrSimGetObjectIntParameter)(simInt objectHandle, simInt parameterID, simInt *parameter);

typedef simInt (__cdecl *ptrSimSetObjectIntParameter)(simInt objectHandle, simInt parameterID, simInt parameter);

typedef simInt (__cdecl *ptrSimGetObjectInt32Parameter)(simInt objectHandle, simInt parameterID, simInt *parameter);

typedef simInt (__cdecl *ptrSimSetObjectInt32Parameter)(simInt objectHandle, simInt parameterID, simInt parameter);

typedef simInt (__cdecl *ptrSimGetObjectFloatParameter)(simInt objectHandle, simInt parameterID, simFloat *parameter);

typedef simInt (__cdecl *ptrSimSetObjectFloatParameter)(simInt objectHandle, simInt parameterID, simFloat parameter);

typedef simChar *(__cdecl *ptrSimGetObjectStringParameter)(simInt objectHandle, simInt parameterID, simInt *parameterLength);

typedef simInt (__cdecl *ptrSimSetObjectStringParameter)(simInt objectHandle, simInt parameterID, simChar *parameter, simInt parameterLength);

typedef simInt (__cdecl *ptrSimSetSimulationPassesPerRenderingPass)(simInt p);

typedef simInt (__cdecl *ptrSimGetRotationAxis)(const simFloat *matrixStart, const simFloat *matrixGoal, simFloat *axis, simFloat *angle);

typedef simInt (__cdecl *ptrSimRotateAroundAxis)(const simFloat *matrixIn, const simFloat *axis, const simFloat *axisPos, simFloat angle, simFloat *matrixOut);

typedef simInt (__cdecl *ptrSimGetJointForce)(simInt jointHandle, simFloat *forceOrTorque);

typedef simInt (__cdecl *ptrSimSetArrayParameter)(simInt parameter, const simVoid *arrayOfValues);

typedef simInt (__cdecl *ptrSimGetArrayParameter)(simInt parameter, simVoid *arrayOfValues);

typedef simInt (__cdecl *ptrSimSetIkGroupProperties)(simInt ikGroupHandle, simInt resolutionMethod, simInt maxIterations, simFloat damping, void *reserved);

typedef simInt (__cdecl *ptrSimSetIkElementProperties)(simInt ikGroupHandle, simInt tipDummyHandle, simInt constraints, const simFloat *precision, const simFloat *weight, void *reserved);

typedef simInt (__cdecl *ptrSimCameraFitToView)(simInt viewHandleOrIndex, simInt objectCount, const simInt *objectHandles, simInt options, simFloat scaling);

typedef simInt (__cdecl *ptrSimPersistentDataWrite)(const simChar *dataName, const simChar *dataValue, simInt dataLength, simInt options);

typedef simChar *(__cdecl *ptrSimPersistentDataRead)(const simChar *dataName, simInt *dataLength);

typedef simInt (__cdecl *ptrSimIsHandleValid)(simInt generalObjectHandle, simInt generalObjectType);

typedef simInt (__cdecl *ptrSimHandleVisionSensor)(simInt visionSensorHandle, simFloat **auxValues, simInt **auxValuesCount);

typedef simInt (__cdecl *ptrSimReadVisionSensor)(simInt visionSensorHandle, simFloat **auxValues, simInt **auxValuesCount);

typedef simInt (__cdecl *ptrSimResetVisionSensor)(simInt visionSensorHandle);

typedef simInt (__cdecl *ptrSimCheckVisionSensor)(simInt visionSensorHandle, simInt entityHandle, simFloat **auxValues, simInt **auxValuesCount);

typedef simFloat *(__cdecl *ptrSimCheckVisionSensorEx)(simInt visionSensorHandle, simInt entityHandle, simBool returnImage);

typedef simInt (__cdecl *ptrSimGetVisionSensorResolution)(simInt visionSensorHandle, simInt *resolution);

typedef simFloat *(__cdecl *ptrSimGetVisionSensorImage)(simInt visionSensorHandle);

typedef simUChar *(__cdecl *ptrSimGetVisionSensorCharImage)(simInt visionSensorHandle, simInt *resolutionX, simInt *resolutionY);

typedef simInt (__cdecl *ptrSimSetVisionSensorImage)(simInt visionSensorHandle, const simFloat *image);

typedef simInt (__cdecl *ptrSimSetVisionSensorCharImage)(simInt visionSensorHandle, const simUChar *image);

typedef simFloat *(__cdecl *ptrSimGetVisionSensorDepthBuffer)(simInt visionSensorHandle);

typedef simInt (__cdecl *ptrSimGetObjectQuaternion)(simInt objectHandle, simInt relativeToObjectHandle, simFloat *quaternion);

typedef simInt (__cdecl *ptrSimSetObjectQuaternion)(simInt objectHandle, simInt relativeToObjectHandle, const simFloat *quaternion);

typedef simInt (__cdecl *ptrSimRMLPosition)(simInt dofs, simDouble timeStep, simInt flags, const simDouble *currentPosVelAccel, const simDouble *maxVelAccelJerk, const simBool *selection, const simDouble *targetPosVel, simDouble *newPosVelAccel, simVoid *auxData);

typedef simInt (__cdecl *ptrSimRMLVelocity)(simInt dofs, simDouble timeStep, simInt flags, const simDouble *currentPosVelAccel, const simDouble *maxAccelJerk, const simBool *selection, const simDouble *targetVel, simDouble *newPosVelAccel, simVoid *auxData);

typedef simInt (__cdecl *ptrSimRMLPos)(simInt dofs, simDouble smallestTimeStep, simInt flags, const simDouble *currentPosVelAccel, const simDouble *maxVelAccelJerk, const simBool *selection, const simDouble *targetPosVel, simVoid *auxData);

typedef simInt (__cdecl *ptrSimRMLVel)(simInt dofs, simDouble smallestTimeStep, simInt flags, const simDouble *currentPosVelAccel, const simDouble *maxAccelJerk, const simBool *selection, const simDouble *targetVel, simVoid *auxData);

typedef simInt (__cdecl *ptrSimRMLStep)(simInt handle, simDouble timeStep, simDouble *newPosVelAccel, simVoid *auxData, simVoid *reserved);

typedef simInt (__cdecl *ptrSimRMLRemove)(simInt handle);

typedef simInt (__cdecl *ptrSimBuildMatrixQ)(const simFloat *position, const simFloat *quaternion, simFloat *matrix);

typedef simInt (__cdecl *ptrSimGetQuaternionFromMatrix)(const simFloat *matrix, simFloat *quaternion);

typedef simChar *(__cdecl *ptrSimFileDialog)(simInt mode, const simChar *title, const simChar *startPath, const simChar *initName, const simChar *extName, const simChar *ext);

typedef simChar *(__cdecl *ptrSimMsgBox)(simInt dlgType, simInt buttons, const simChar *title, const simChar *message);

typedef simInt (__cdecl *ptrSimSetShapeMassAndInertia)(simInt shapeHandle, simFloat mass, const simFloat *inertiaMatrix, const simFloat *centerOfMass, const simFloat *transformation);

typedef simInt (__cdecl *ptrSimGetShapeMassAndInertia)(simInt shapeHandle, simFloat *mass, simFloat *inertiaMatrix, simFloat *centerOfMass, const simFloat *transformation);

typedef simInt (__cdecl *ptrSimGroupShapes)(const simInt *shapeHandles, simInt shapeCount);

typedef simInt *(__cdecl *ptrSimUngroupShape)(simInt shapeHandle, simInt *shapeCount);

typedef simInt (__cdecl *ptrSimConvexDecompose)(simInt shapeHandle, simInt options, const simInt *intParams, const simFloat *floatParams);

typedef simFloat *(__cdecl *ptrSimGetIkGroupMatrix)(simInt ikGroupHandle, simInt options, simInt *matrixSize);

typedef simInt (__cdecl *ptrSimAddGhost)(simInt ghostGroup, simInt objectHandle, simInt options, simFloat startTime, simFloat endTime, const simFloat *color);

typedef simInt (__cdecl *ptrSimModifyGhost)(simInt ghostGroup, simInt ghostId, simInt operation, simFloat floatValue, simInt options, simInt optionsMask, const simFloat *colorOrTransformation);

typedef simVoid (__cdecl *ptrSimQuitSimulator)(simBool doNotDisplayMessages);

typedef simInt (__cdecl *ptrSimGetThreadId)();

typedef simInt (__cdecl *ptrSimLockResources)(simInt lockType, simInt reserved);

typedef simInt (__cdecl *ptrSimUnlockResources)(simInt lockHandle);

typedef simInt (__cdecl *ptrSimEnableEventCallback)(simInt eventCallbackType, const simChar *plugin, simInt reserved);

typedef simInt (__cdecl *ptrSimSetShapeMaterial)(simInt shapeHandle, simInt materialIdOrShapeHandle);

typedef simInt (__cdecl *ptrSimGetTextureId)(const simChar *textureName, simInt *resolution);

typedef simChar *(__cdecl *ptrSimReadTexture)(simInt textureId, simInt options, simInt posX, simInt posY, simInt sizeX, simInt sizeY);

typedef simInt (__cdecl *ptrSimWriteTexture)(simInt textureId, simInt options, const simChar *data, simInt posX, simInt posY, simInt sizeX, simInt sizeY, simFloat interpol);

typedef simInt (__cdecl *ptrSimCreateTexture)(const simChar *fileName, simInt options, const simFloat *planeSizes, const simFloat *scalingUV, const simFloat *xy_g, simInt fixedResolution, simInt *textureId, simInt *resolution, const simVoid *reserved);

typedef simInt (__cdecl *ptrSimWriteCustomDataBlock)(simInt objectHandle, const simChar *tagName, const simChar *data, simInt dataSize);

typedef simChar *(__cdecl *ptrSimReadCustomDataBlock)(simInt objectHandle, const simChar *tagName, simInt *dataSize);

typedef simChar *(__cdecl *ptrSimReadCustomDataBlockTags)(simInt objectHandle, simInt *tagCount);

typedef simInt (__cdecl *ptrSimAddPointCloud)(simInt pageMask, simInt layerMask, simInt objectHandle, simInt options, simFloat pointSize, simInt ptCnt, const simFloat *pointCoordinates, const simChar *defaultColors, const simChar *pointColors, const simFloat *pointNormals);

typedef simInt (__cdecl *ptrSimModifyPointCloud)(simInt pointCloudHandle, simInt operation, const simInt *intParam, const simFloat *floatParam);

typedef simInt (__cdecl *ptrSimGetShapeGeomInfo)(simInt shapeHandle, simInt *intData, simFloat *floatData, simVoid *reserved);

typedef simInt *(__cdecl *ptrSimGetObjectsInTree)(simInt treeBaseHandle, simInt objectType, simInt options, simInt *objectCount);

typedef simInt (__cdecl *ptrSimSetObjectSizeValues)(simInt objectHandle, const simFloat *sizeValues);

typedef simInt (__cdecl *ptrSimGetObjectSizeValues)(simInt objectHandle, simFloat *sizeValues);

typedef simInt (__cdecl *ptrSimScaleObject)(simInt objectHandle, simFloat xScale, simFloat yScale, simFloat zScale, simInt options);

typedef simInt (__cdecl *ptrSimSetShapeTexture)(simInt shapeHandle, simInt textureId, simInt mappingMode, simInt options, const simFloat *uvScaling, const simFloat *position, const simFloat *orientation);

typedef simInt (__cdecl *ptrSimGetShapeTextureId)(simInt shapeHandle);

typedef simInt *(__cdecl *ptrSimGetCollectionObjects)(simInt collectionHandle, simInt *objectCount);

typedef simInt (__cdecl *ptrSimHandleCustomizationScripts)(simInt callType);

typedef simInt (__cdecl *ptrSimSetScriptAttribute)(simInt scriptHandle, simInt attributeID, simFloat floatVal, simInt intOrBoolVal);

typedef simInt (__cdecl *ptrSimGetScriptAttribute)(simInt scriptHandle, simInt attributeID, simFloat *floatVal, simInt *intOrBoolVal);

typedef simInt (__cdecl *ptrSimReorientShapeBoundingBox)(simInt shapeHandle, simInt relativeToHandle, simInt reservedSetToZero);

typedef simInt (__cdecl *ptrSimSwitchThread)();

typedef simInt (__cdecl *ptrSimCreateIkGroup)(simInt options, const simInt *intParams, const simFloat *floatParams, const simVoid *reserved);

typedef simInt (__cdecl *ptrSimRemoveIkGroup)(simInt ikGroupHandle);

typedef simInt (__cdecl *ptrSimCreateIkElement)(simInt ikGroupHandle, simInt options, const simInt *intParams, const simFloat *floatParams, const simVoid *reserved);

typedef simInt (__cdecl *ptrSimCreateCollection)(const simChar *collectionName, simInt options);

typedef simInt (__cdecl *ptrSimAddObjectToCollection)(simInt collectionHandle, simInt objectHandle, simInt what, simInt options);

typedef simInt (__cdecl *ptrSimSaveImage)(const simUChar *image, const simInt *resolution, simInt options, const simChar *filename, simInt quality, simVoid *reserved);

typedef simUChar *(__cdecl *ptrSimLoadImage)(simInt *resolution, simInt options, const simChar *filename, simVoid *reserved);

typedef simUChar *(__cdecl *ptrSimGetScaledImage)(const simUChar *imageIn, const simInt *resolutionIn, simInt *resolutionOut, simInt options, simVoid *reserved);

typedef simInt (__cdecl *ptrSimTransformImage)(simUChar *image, const simInt *resolution, simInt options, const simFloat *floatParams, const simInt *intParams, simVoid *reserved);

typedef simInt (__cdecl *ptrSimGetQHull)(const simFloat *inVertices, simInt inVerticesL, simFloat **verticesOut, simInt *verticesOutL, simInt **indicesOut, simInt *indicesOutL, simInt reserved1, const simFloat *reserved2);

typedef simInt (__cdecl *ptrSimGetDecimatedMesh)(const simFloat *inVertices, simInt inVerticesL, const simInt *inIndices, simInt inIndicesL, simFloat **verticesOut, simInt *verticesOutL, simInt **indicesOut, simInt *indicesOutL, simFloat decimationPercent, simInt reserved1, const simFloat *reserved2);

typedef simInt (__cdecl *ptrSimExportIk)(const simChar *pathAndFilename, simInt reserved1, simVoid *reserved2);

typedef simInt (__cdecl *ptrSimCallScriptFunction)(simInt scriptHandleOrType, const simChar *functionNameAtScriptName, SLuaCallBack *data, const simChar *reservedSetToNull);

typedef simInt (__cdecl *ptrSimCallScriptFunctionEx)(simInt scriptHandleOrType, const simChar *functionNameAtScriptName, simInt stackId);

typedef simInt (__cdecl *ptrSimComputeJacobian)(simInt ikGroupHandle, simInt options, simVoid *reserved);

typedef simInt (__cdecl *ptrSimGetConfigForTipPose)(simInt ikGroupHandle, simInt jointCnt, const simInt *jointHandles, simFloat thresholdDist, simInt maxTimeInMs, simFloat *retConfig, const simFloat *metric, simInt collisionPairCnt, const simInt *collisionPairs, const simInt *jointOptions, const simFloat *lowLimits, const simFloat *ranges, simVoid *reserved);

typedef simFloat *(__cdecl *ptrSimGenerateIkPath)(simInt ikGroupHandle, simInt jointCnt, const simInt *jointHandles, simInt ptCnt, simInt collisionPairCnt, const simInt *collisionPairs, const simInt *jointOptions, simVoid *reserved);

typedef simChar *(__cdecl *ptrSimGetExtensionString)(simInt objectHandle, simInt index, const char *key);

typedef simInt (__cdecl *ptrSimComputeMassAndInertia)(simInt shapeHandle, simFloat density);

typedef simInt (__cdecl *ptrSimCreateStack)();

typedef simInt (__cdecl *ptrSimReleaseStack)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimCopyStack)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimPushNullOntoStack)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimPushBoolOntoStack)(simInt stackHandle, simBool value);

typedef simInt (__cdecl *ptrSimPushInt32OntoStack)(simInt stackHandle, simInt value);

typedef simInt (__cdecl *ptrSimPushFloatOntoStack)(simInt stackHandle, simFloat value);

typedef simInt (__cdecl *ptrSimPushDoubleOntoStack)(simInt stackHandle, simDouble value);

typedef simInt (__cdecl *ptrSimPushStringOntoStack)(simInt stackHandle, const simChar *value, simInt stringSize);

typedef simInt (__cdecl *ptrSimPushUInt8TableOntoStack)(simInt stackHandle, const simUChar *values, simInt valueCnt);

typedef simInt (__cdecl *ptrSimPushInt32TableOntoStack)(simInt stackHandle, const simInt *values, simInt valueCnt);

typedef simInt (__cdecl *ptrSimPushFloatTableOntoStack)(simInt stackHandle, const simFloat *values, simInt valueCnt);

typedef simInt (__cdecl *ptrSimPushDoubleTableOntoStack)(simInt stackHandle, const simDouble *values, simInt valueCnt);

typedef simInt (__cdecl *ptrSimPushTableOntoStack)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimInsertDataIntoStackTable)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimGetStackSize)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimPopStackItem)(simInt stackHandle, simInt count);

typedef simInt (__cdecl *ptrSimMoveStackItemToTop)(simInt stackHandle, simInt cIndex);

typedef simInt (__cdecl *ptrSimIsStackValueNull)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimGetStackBoolValue)(simInt stackHandle, simBool *boolValue);

typedef simInt (__cdecl *ptrSimGetStackInt32Value)(simInt stackHandle, simInt *numberValue);

typedef simInt (__cdecl *ptrSimGetStackFloatValue)(simInt stackHandle, simFloat *numberValue);

typedef simInt (__cdecl *ptrSimGetStackDoubleValue)(simInt stackHandle, simDouble *numberValue);

typedef simChar *(__cdecl *ptrSimGetStackStringValue)(simInt stackHandle, simInt *stringSize);

typedef simInt (__cdecl *ptrSimGetStackTableInfo)(simInt stackHandle, simInt infoType);

typedef simInt (__cdecl *ptrSimGetStackUInt8Table)(simInt stackHandle, simUChar *array, simInt count);

typedef simInt (__cdecl *ptrSimGetStackInt32Table)(simInt stackHandle, simInt *array, simInt count);

typedef simInt (__cdecl *ptrSimGetStackFloatTable)(simInt stackHandle, simFloat *array, simInt count);

typedef simInt (__cdecl *ptrSimGetStackDoubleTable)(simInt stackHandle, simDouble *array, simInt count);

typedef simInt (__cdecl *ptrSimUnfoldStackTable)(simInt stackHandle);

typedef simInt (__cdecl *ptrSimDebugStack)(simInt stackHandle, simInt cIndex);

typedef simInt (__cdecl *ptrSimSetScriptVariable)(simInt scriptHandleOrType, const simChar *variableNameAtScriptName, simInt stackHandle);

typedef simFloat (__cdecl *ptrSimGetEngineFloatParameter)(simInt paramId, simInt objectHandle, const simVoid *object, simBool *ok);

typedef simInt (__cdecl *ptrSimGetEngineInt32Parameter)(simInt paramId, simInt objectHandle, const simVoid *object, simBool *ok);

typedef simBool (__cdecl *ptrSimGetEngineBoolParameter)(simInt paramId, simInt objectHandle, const simVoid *object, simBool *ok);

typedef simInt (__cdecl *ptrSimSetEngineFloatParameter)(simInt paramId, simInt objectHandle, const simVoid *object, simFloat val);

typedef simInt (__cdecl *ptrSimSetEngineInt32Parameter)(simInt paramId, simInt objectHandle, const simVoid *object, simInt val);

typedef simInt (__cdecl *ptrSimSetEngineBoolParameter)(simInt paramId, simInt objectHandle, const simVoid *object, simBool val);

typedef simInt (__cdecl *ptrSimCreateOctree)(simFloat voxelSize, simInt options, simFloat pointSize, simVoid *reserved);

typedef simInt (__cdecl *ptrSimCreatePointCloud)(simFloat maxVoxelSize, simInt maxPtCntPerVoxel, simInt options, simFloat pointSize, simVoid *reserved);

typedef simInt (__cdecl *ptrSimSetPointCloudOptions)(simInt pointCloudHandle, simFloat maxVoxelSize, simInt maxPtCntPerVoxel, simInt options, simFloat pointSize, simVoid *reserved);

typedef simInt (__cdecl *ptrSimGetPointCloudOptions)(simInt pointCloudHandle, simFloat *maxVoxelSize, simInt *maxPtCntPerVoxel, simInt *options, simFloat *pointSize, simVoid *reserved);

typedef simInt (__cdecl *ptrSimInsertVoxelsIntoOctree)(simInt octreeHandle, simInt options, const simFloat *pts, simInt ptCnt, const simUChar *color, const simUInt *tag, simVoid *reserved);

typedef simInt (__cdecl *ptrSimRemoveVoxelsFromOctree)(simInt octreeHandle, simInt options, const simFloat *pts, simInt ptCnt, simVoid *reserved);

typedef simInt (__cdecl *ptrSimInsertPointsIntoPointCloud)(simInt pointCloudHandle, simInt options, const simFloat *pts, simInt ptCnt, const simUChar *color, simVoid *optionalValues);

typedef simInt (__cdecl *ptrSimRemovePointsFromPointCloud)(simInt pointCloudHandle, simInt options, const simFloat *pts, simInt ptCnt, simFloat tolerance, simVoid *reserved);

typedef simInt (__cdecl *ptrSimIntersectPointsWithPointCloud)(simInt pointCloudHandle, simInt options, const simFloat *pts, simInt ptCnt, simFloat tolerance, simVoid *reserved);

typedef const float *(__cdecl *ptrSimGetOctreeVoxels)(simInt octreeHandle, simInt *ptCnt, simVoid *reserved);

typedef const float *(__cdecl *ptrSimGetPointCloudPoints)(simInt pointCloudHandle, simInt *ptCnt, simVoid *reserved);

typedef simInt (__cdecl *ptrSimInsertObjectIntoOctree)(simInt octreeHandle, simInt objectHandle, simInt options, const simUChar *color, simUInt tag, simVoid *reserved);

typedef simInt (__cdecl *ptrSimSubtractObjectFromOctree)(simInt octreeHandle, simInt objectHandle, simInt options, simVoid *reserved);

typedef simInt (__cdecl *ptrSimInsertObjectIntoPointCloud)(simInt pointCloudHandle, simInt objectHandle, simInt options, simFloat gridSize, const simUChar *color, simVoid *optionalValues);

typedef simInt (__cdecl *ptrSimSubtractObjectFromPointCloud)(simInt pointCloudHandle, simInt objectHandle, simInt options, simFloat tolerance, simVoid *reserved);

typedef simInt (__cdecl *ptrSimCheckOctreePointOccupancy)(simInt octreeHandle, simInt options, const simFloat *points, simInt ptCnt, simUInt *tag, simUInt64 *location, simVoid *reserved);

typedef simChar *(__cdecl *ptrSimOpenTextEditor)(const simChar *initText, const simChar *xml, simInt *various);

typedef simChar *(__cdecl *ptrSimPackTable)(simInt stackHandle, simInt *bufferSize);

typedef simInt (__cdecl *ptrSimUnpackTable)(simInt stackHandle, const simChar *buffer, simInt bufferSize);

typedef simInt (__cdecl *ptrSimSetVisionSensorFilter)(simInt visionSensorHandle, simInt filterIndex, simInt options, const simInt *pSizes, const simUChar *bytes, const simInt *ints, const simFloat *floats, const simUChar *custom);

typedef simInt (__cdecl *ptrSimGetVisionSensorFilter)(simInt visionSensorHandle, simInt filterIndex, simInt *options, simInt *pSizes, simUChar **bytes, simInt **ints, simFloat **floats, simUChar **custom);

typedef simInt (__cdecl *ptrSimSetReferencedHandles)(simInt objectHandle, simInt count, const simInt *referencedHandles, const simInt *reserved1, const simInt *reserved2);

typedef simInt (__cdecl *ptrSimGetReferencedHandles)(simInt objectHandle, simInt **referencedHandles, simInt **reserved1, simInt **reserved2);

typedef simInt (__cdecl *ptrSimGetShapeViz)(simInt shapeHandle, simInt index, struct SShapeVizInfo *info);

typedef simInt (__cdecl *ptrSimExecuteScriptString)(simInt scriptHandleOrType, const simChar *stringAtScriptName, simInt stackHandle);

typedef simChar *(__cdecl *ptrSimGetApiFunc)(simInt scriptHandleOrType, const simChar *apiWord);

typedef simChar *(__cdecl *ptrSimGetApiInfo)(simInt scriptHandleOrType, const simChar *apiWord);

typedef simInt (__cdecl *ptrSimSetModuleInfo)(const simChar *moduleName, simInt infoType, const simChar *stringInfo, simInt intInfo);

typedef simInt (__cdecl *ptrSimGetModuleInfo)(const simChar *moduleName, simInt infoType, simChar **stringInfo, simInt *intInfo);


typedef simInt (__cdecl *ptr_simGetContactCallbackCount)();

typedef const void *(__cdecl *ptr_simGetContactCallback)(simInt index);

typedef simVoid (__cdecl *ptr_simSetDynamicSimulationIconCode)(simVoid *object, simInt code);

typedef simVoid (__cdecl *ptr_simSetDynamicObjectFlagForVisualization)(simVoid *object, simInt flag);

typedef simInt (__cdecl *ptr_simGetObjectListSize)(simInt objType);

typedef const simVoid *(__cdecl *ptr_simGetObjectFromIndex)(simInt objType, simInt index);

typedef simInt (__cdecl *ptr_simGetObjectID)(const simVoid *object);

typedef simInt (__cdecl *ptr_simGetObjectType)(const simVoid *object);

typedef const simVoid **(__cdecl *ptr_simGetObjectChildren)(const simVoid *object, simInt *count);

typedef const simVoid *(__cdecl *ptr_simGetGeomProxyFromShape)(const simVoid *shape);

typedef const simVoid *(__cdecl *ptr_simGetParentObject)(const simVoid *object);

typedef const simVoid *(__cdecl *ptr_simGetObject)(int objID);

typedef const simVoid *(__cdecl *ptr_simGetIkGroupObject)(int ikGroupID);

typedef simInt (__cdecl *ptr_simMpHandleIkGroupObject)(const simVoid *ikGroup);

typedef simVoid (__cdecl *ptr_simGetObjectLocalTransformation)(const simVoid *object, simFloat *pos, simFloat *quat, simBool excludeFirstJointTransformation);

typedef simVoid (__cdecl *ptr_simSetObjectLocalTransformation)(simVoid *object, const simFloat *pos, const simFloat *quat);

typedef simVoid (__cdecl *ptr_simSetObjectCumulativeTransformation)(simVoid *object, const simFloat *pos, const simFloat *quat, simBool keepChildrenInPlace);

typedef simVoid (__cdecl *ptr_simGetObjectCumulativeTransformation)(const simVoid *object, simFloat *pos, simFloat *quat, simBool excludeFirstJointTransformation);

typedef simBool (__cdecl *ptr_simIsShapeDynamicallyStatic)(const simVoid *shape);

typedef simInt (__cdecl *ptr_simGetTreeDynamicProperty)(const simVoid *object);

typedef simInt (__cdecl *ptr_simGetDummyLinkType)(const simVoid *dummy, simInt *linkedDummyID);

typedef simInt (__cdecl *ptr_simGetJointMode)(const simVoid *joint);

typedef simBool (__cdecl *ptr_simIsJointInHybridOperation)(const simVoid *joint);

typedef simVoid (__cdecl *ptr_simDisableDynamicTreeForManipulation)(const simVoid *object, simBool disableFlag);

typedef simBool (__cdecl *ptr_simIsShapeDynamicallyRespondable)(const simVoid *shape);

typedef simInt (__cdecl *ptr_simGetDynamicCollisionMask)(const simVoid *shape);

typedef const simVoid *(__cdecl *ptr_simGetLastParentForLocalGlobalCollidable)(const simVoid *shape);

typedef simVoid (__cdecl *ptr_simSetShapeIsStaticAndNotRespondableButDynamicTag)(const simVoid *shape, simBool tag);

typedef simBool (__cdecl *ptr_simGetShapeIsStaticAndNotRespondableButDynamicTag)(const simVoid *shape);

typedef simVoid (__cdecl *ptr_simSetJointPosition)(const simVoid *joint, simFloat pos);

typedef simFloat (__cdecl *ptr_simGetJointPosition)(const simVoid *joint);

typedef simVoid (__cdecl *ptr_simSetDynamicMotorPositionControlTargetPosition)(const simVoid *joint, simFloat pos);

typedef simVoid (__cdecl *ptr_simGetInitialDynamicVelocity)(const simVoid *shape, simFloat *vel);

typedef simVoid (__cdecl *ptr_simSetInitialDynamicVelocity)(simVoid *shape, const simFloat *vel);

typedef simVoid (__cdecl *ptr_simGetInitialDynamicAngVelocity)(const simVoid *shape, simFloat *angularVel);

typedef simVoid (__cdecl *ptr_simSetInitialDynamicAngVelocity)(simVoid *shape, const simFloat *angularVel);

typedef simBool (__cdecl *ptr_simGetStartSleeping)(const simVoid *shape);

typedef simBool (__cdecl *ptr_simGetWasPutToSleepOnce)(const simVoid *shape);

typedef simBool (__cdecl *ptr_simGetDynamicsFullRefreshFlag)(const simVoid *object);

typedef simVoid (__cdecl *ptr_simSetDynamicsFullRefreshFlag)(const simVoid *object, simBool flag);

typedef simVoid (__cdecl *ptr_simSetGeomProxyDynamicsFullRefreshFlag)(simVoid *geomData, simBool flag);

typedef simBool (__cdecl *ptr_simGetGeomProxyDynamicsFullRefreshFlag)(const simVoid *geomData);

typedef simBool (__cdecl *ptr_simGetParentFollowsDynamic)(const simVoid *shape);

typedef simVoid (__cdecl *ptr_simSetShapeDynamicVelocity)(simVoid *shape, const simFloat *linear, const simFloat *angular);

typedef simVoid (__cdecl *ptr_simGetAdditionalForceAndTorque)(const simVoid *shape, simFloat *force, simFloat *torque);

typedef simVoid (__cdecl *ptr_simClearAdditionalForceAndTorque)(const simVoid *shape);

typedef simBool (__cdecl *ptr_simGetJointPositionInterval)(const simVoid *joint, simFloat *minValue, simFloat *rangeValue);

typedef simInt (__cdecl *ptr_simGetJointType)(const simVoid *joint);

typedef simBool (__cdecl *ptr_simIsForceSensorBroken)(const simVoid *forceSensor);

typedef simVoid (__cdecl *ptr_simGetDynamicForceSensorLocalTransformationPart2)(const simVoid *forceSensor, simFloat *pos, simFloat *quat);

typedef simBool (__cdecl *ptr_simIsDynamicMotorEnabled)(const simVoid *joint);

typedef simBool (__cdecl *ptr_simIsDynamicMotorPositionCtrlEnabled)(const simVoid *joint);

typedef simBool (__cdecl *ptr_simIsDynamicMotorTorqueModulationEnabled)(const simVoid *joint);

typedef simVoid (__cdecl *ptr_simGetMotorPid)(const simVoid *joint, simFloat *pParam, simFloat *iParam, simFloat *dParam);

typedef simFloat (__cdecl *ptr_simGetDynamicMotorTargetPosition)(const simVoid *joint);

typedef simFloat (__cdecl *ptr_simGetDynamicMotorTargetVelocity)(const simVoid *joint);

typedef simFloat (__cdecl *ptr_simGetDynamicMotorMaxForce)(const simVoid *joint);

typedef simFloat (__cdecl *ptr_simGetDynamicMotorUpperLimitVelocity)(const simVoid *joint);

typedef simVoid (__cdecl *ptr_simSetDynamicMotorReflectedPositionFromDynamicEngine)(simVoid *joint, simFloat pos);

typedef simVoid (__cdecl *ptr_simSetJointSphericalTransformation)(simVoid *joint, const simFloat *quat);

typedef simVoid (__cdecl *ptr_simAddForceSensorCumulativeForcesAndTorques)(simVoid *forceSensor, const simFloat *force, const simFloat *torque, int totalPassesCount);

typedef simVoid (__cdecl *ptr_simAddJointCumulativeForcesOrTorques)(simVoid *joint, simFloat forceOrTorque, int totalPassesCount);

typedef simVoid (__cdecl *ptr_simSetDynamicJointLocalTransformationPart2)(simVoid *joint, const simFloat *pos, const simFloat *quat);

typedef simVoid (__cdecl *ptr_simSetDynamicForceSensorLocalTransformationPart2)(simVoid *forceSensor, const simFloat *pos, const simFloat *quat);

typedef simVoid (__cdecl *ptr_simSetDynamicJointLocalTransformationPart2IsValid)(simVoid *joint, simBool valid);

typedef simVoid (__cdecl *ptr_simSetDynamicForceSensorLocalTransformationPart2IsValid)(simVoid *forceSensor, simBool valid);

typedef const simVoid *(__cdecl *ptr_simGetGeomWrapFromGeomProxy)(const simVoid *geomData);

typedef simVoid (__cdecl *ptr_simGetLocalInertiaFrame)(const simVoid *geomInfo, simFloat *pos, simFloat *quat);

typedef simInt (__cdecl *ptr_simGetPurePrimitiveType)(const simVoid *geomInfo);

typedef simBool (__cdecl *ptr_simIsGeomWrapGeometric)(const simVoid *geomInfo);

typedef simBool (__cdecl *ptr_simIsGeomWrapConvex)(const simVoid *geomInfo);

typedef simInt (__cdecl *ptr_simGetGeometricCount)(const simVoid *geomInfo);

typedef simVoid (__cdecl *ptr_simGetAllGeometrics)(const simVoid *geomInfo, simVoid **allGeometrics);

typedef simVoid (__cdecl *ptr_simGetPurePrimitiveSizes)(const simVoid *geometric, simFloat *sizes);

typedef simVoid (__cdecl *ptr_simMakeDynamicAnnouncement)(int announceType);

typedef simVoid (__cdecl *ptr_simGetVerticesLocalFrame)(const simVoid *geometric, simFloat *pos, simFloat *quat);

typedef const simFloat *(__cdecl *ptr_simGetHeightfieldData)(const simVoid *geometric, simInt *xCount, simInt *yCount, simFloat *minHeight, simFloat *maxHeight);

typedef simVoid (__cdecl *ptr_simGetCumulativeMeshes)(const simVoid *geomInfo, simFloat **vertices, simInt *verticesSize, simInt **indices, simInt *indicesSize);

typedef simFloat (__cdecl *ptr_simGetMass)(const simVoid *geomInfo);

typedef simVoid (__cdecl *ptr_simGetPrincipalMomentOfInertia)(const simVoid *geomInfo, simFloat *inertia);

typedef simVoid (__cdecl *ptr_simGetGravity)(simFloat *gravity);

typedef simInt (__cdecl *ptr_simGetTimeDiffInMs)(simInt previousTime);

typedef simBool (__cdecl *ptr_simDoEntitiesCollide)(simInt entity1ID, simInt entity2ID, simInt *cacheBuffer, simBool overrideCollidableFlagIfShape1, simBool overrideCollidableFlagIfShape2, simBool pathPlanningRoutineCalling);

typedef simBool (__cdecl *ptr_simGetDistanceBetweenEntitiesIfSmaller)(simInt entity1ID, simInt entity2ID, simFloat *distance, simFloat *ray, simInt *cacheBuffer, simBool overrideMeasurableFlagIfNonCollection1, simBool overrideMeasurableFlagIfNonCollection2, simBool pathPlanningRoutineCalling);

typedef simInt (__cdecl *ptr_simHandleJointControl)(const simVoid *joint, simInt auxV, const simInt *inputValuesInt, const simFloat *inputValuesFloat, simFloat *outputValues);

typedef simInt (__cdecl *ptr_simHandleCustomContact)(simInt objHandle1, simInt objHandle2, simInt engine, simInt *dataInt, simFloat *dataFloat);

typedef simFloat (__cdecl *ptr_simGetPureHollowScaling)(const simVoid *geometric);

typedef simInt (__cdecl *ptr_simGetJointCallbackCallOrder)(const simVoid *joint);

typedef simVoid (__cdecl *ptr_simDynCallback)(const simInt *intData, const simFloat *floatData);


extern ptrSimRunSimulator simRunSimulator;
extern ptrSimGetSimulatorMessage simGetSimulatorMessage;
extern ptrSimGetMainWindow simGetMainWindow;
extern ptrSimGetLastError simGetLastError;
extern ptrSimLoadModule simLoadModule;
extern ptrSimUnloadModule simUnloadModule;
extern ptrSimSendModuleMessage simSendModuleMessage;
extern ptrSimSetBooleanParameter simSetBooleanParameter;
extern ptrSimGetBooleanParameter simGetBooleanParameter;
extern ptrSimSetBoolParameter simSetBoolParameter;
extern ptrSimGetBoolParameter simGetBoolParameter;
extern ptrSimSetIntegerParameter simSetIntegerParameter;
extern ptrSimGetIntegerParameter simGetIntegerParameter;
extern ptrSimSetInt32Parameter simSetInt32Parameter;
extern ptrSimGetInt32Parameter simGetInt32Parameter;
extern ptrSimGetUInt64Parameter simGetUInt64Parameter;
extern ptrSimSetFloatingParameter simSetFloatingParameter;
extern ptrSimGetFloatingParameter simGetFloatingParameter;
extern ptrSimSetFloatParameter simSetFloatParameter;
extern ptrSimGetFloatParameter simGetFloatParameter;
extern ptrSimSetStringParameter simSetStringParameter;
extern ptrSimGetStringParameter simGetStringParameter;
extern ptrSimGetObjectHandle simGetObjectHandle;
extern ptrSimRemoveObject simRemoveObject;
extern ptrSimRemoveModel simRemoveModel;
extern ptrSimGetObjectName simGetObjectName;
extern ptrSimGetObjects simGetObjects;
extern ptrSimSetObjectName simSetObjectName;
extern ptrSimGetCollectionHandle simGetCollectionHandle;
extern ptrSimRemoveCollection simRemoveCollection;
extern ptrSimEmptyCollection simEmptyCollection;
extern ptrSimGetCollectionName simGetCollectionName;
extern ptrSimSetCollectionName simSetCollectionName;
extern ptrSimGetObjectMatrix simGetObjectMatrix;
extern ptrSimSetObjectMatrix simSetObjectMatrix;
extern ptrSimGetObjectPosition simGetObjectPosition;
extern ptrSimSetObjectPosition simSetObjectPosition;
extern ptrSimGetObjectOrientation simGetObjectOrientation;
extern ptrSimSetObjectOrientation simSetObjectOrientation;
extern ptrSimGetJointPosition simGetJointPosition;
extern ptrSimSetJointPosition simSetJointPosition;
extern ptrSimSetJointTargetPosition simSetJointTargetPosition;
extern ptrSimGetJointTargetPosition simGetJointTargetPosition;
extern ptrSimSetJointForce simSetJointForce;
extern ptrSimGetPathPosition simGetPathPosition;
extern ptrSimSetPathPosition simSetPathPosition;
extern ptrSimGetPathLength simGetPathLength;
extern ptrSimGetJointMatrix simGetJointMatrix;
extern ptrSimSetSphericalJointMatrix simSetSphericalJointMatrix;
extern ptrSimGetJointInterval simGetJointInterval;
extern ptrSimSetJointInterval simSetJointInterval;
extern ptrSimGetObjectParent simGetObjectParent;
extern ptrSimGetObjectChild simGetObjectChild;
extern ptrSimSetObjectParent simSetObjectParent;
extern ptrSimGetObjectType simGetObjectType;
extern ptrSimGetJointType simGetJointType;
extern ptrSimBuildIdentityMatrix simBuildIdentityMatrix;
extern ptrSimCopyMatrix simCopyMatrix;
extern ptrSimBuildMatrix simBuildMatrix;
extern ptrSimGetEulerAnglesFromMatrix simGetEulerAnglesFromMatrix;
extern ptrSimInvertMatrix simInvertMatrix;
extern ptrSimMultiplyMatrices simMultiplyMatrices;
extern ptrSimInterpolateMatrices simInterpolateMatrices;
extern ptrSimTransformVector simTransformVector;
extern ptrSimReservedCommand simReservedCommand;
extern ptrSimGetSimulationTime simGetSimulationTime;
extern ptrSimGetSimulationState simGetSimulationState;
extern ptrSimGetSystemTime simGetSystemTime;
extern ptrSimGetSystemTimeInMilliseconds simGetSystemTimeInMilliseconds;
extern ptrSimGetSystemTimeInMs simGetSystemTimeInMs;
extern ptrSimLoadScene simLoadScene;
extern ptrSimCloseScene simCloseScene;
extern ptrSimSaveScene simSaveScene;
extern ptrSimLoadModel simLoadModel;
extern ptrSimSaveModel simSaveModel;
extern ptrSimAddStatusbarMessage simAddStatusbarMessage;
extern ptrSimAddModuleMenuEntry simAddModuleMenuEntry;
extern ptrSimSetModuleMenuItemState simSetModuleMenuItemState;
extern ptrSimDoesFileExist simDoesFileExist;
extern ptrSimIsObjectInSelection simIsObjectInSelection;
extern ptrSimAddObjectToSelection simAddObjectToSelection;
extern ptrSimRemoveObjectFromSelection simRemoveObjectFromSelection;
extern ptrSimGetObjectSelectionSize simGetObjectSelectionSize;
extern ptrSimGetObjectLastSelection simGetObjectLastSelection;
extern ptrSimGetObjectSelection simGetObjectSelection;
extern ptrSimHandleCollision simHandleCollision;
extern ptrSimReadCollision simReadCollision;
extern ptrSimHandleDistance simHandleDistance;
extern ptrSimReadDistance simReadDistance;
extern ptrSimHandleProximitySensor simHandleProximitySensor;
extern ptrSimReadProximitySensor simReadProximitySensor;
extern ptrSimHandleMill simHandleMill;
extern ptrSimHandleIkGroup simHandleIkGroup;
extern ptrSimCheckIkGroup simCheckIkGroup;
extern ptrSimHandleDynamics simHandleDynamics;
extern ptrSimGetMechanismHandle simGetMechanismHandle;
extern ptrSimHandleMechanism simHandleMechanism;
extern ptrSimGetScriptHandle simGetScriptHandle;
extern ptrSimSetScriptText simSetScriptText;
extern ptrSimGetScriptText simGetScriptText;
extern ptrSimGetScriptProperty simGetScriptProperty;
extern ptrSimAssociateScriptWithObject simAssociateScriptWithObject;
extern ptrSimGetScript simGetScript;
extern ptrSimGetScriptAssociatedWithObject simGetScriptAssociatedWithObject;
extern ptrSimGetCustomizationScriptAssociatedWithObject simGetCustomizationScriptAssociatedWithObject;
extern ptrSimGetObjectAssociatedWithScript simGetObjectAssociatedWithScript;
extern ptrSimGetScriptName simGetScriptName;
extern ptrSimHandleMainScript simHandleMainScript;
extern ptrSimResetScript simResetScript;
extern ptrSimAddScript simAddScript;
extern ptrSimRemoveScript simRemoveScript;
extern ptrSimRefreshDialogs simRefreshDialogs;
extern ptrSimGetCollisionHandle simGetCollisionHandle;
extern ptrSimGetDistanceHandle simGetDistanceHandle;
extern ptrSimGetIkGroupHandle simGetIkGroupHandle;
extern ptrSimResetCollision simResetCollision;
extern ptrSimResetDistance simResetDistance;
extern ptrSimResetProximitySensor simResetProximitySensor;
extern ptrSimResetMill simResetMill;
extern ptrSimCheckProximitySensor simCheckProximitySensor;
extern ptrSimCheckProximitySensorEx simCheckProximitySensorEx;
extern ptrSimCheckProximitySensorEx2 simCheckProximitySensorEx2;
extern ptrSimCreateBuffer simCreateBuffer;
extern ptrSimReleaseBuffer simReleaseBuffer;
extern ptrSimCheckCollision simCheckCollision;
extern ptrSimCheckCollisionEx simCheckCollisionEx;
extern ptrSimCheckDistance simCheckDistance;
extern ptrSimGetObjectConfiguration simGetObjectConfiguration;
extern ptrSimSetObjectConfiguration simSetObjectConfiguration;
extern ptrSimGetConfigurationTree simGetConfigurationTree;
extern ptrSimSetConfigurationTree simSetConfigurationTree;
extern ptrSimSetSimulationTimeStep simSetSimulationTimeStep;
extern ptrSimGetSimulationTimeStep simGetSimulationTimeStep;
extern ptrSimGetRealTimeSimulation simGetRealTimeSimulation;
extern ptrSimIsRealTimeSimulationStepNeeded simIsRealTimeSimulationStepNeeded;
extern ptrSimAdjustRealTimeTimer simAdjustRealTimeTimer;
extern ptrSimGetSimulationPassesPerRenderingPass simGetSimulationPassesPerRenderingPass;
extern ptrSimAdvanceSimulationByOneStep simAdvanceSimulationByOneStep;
extern ptrSimStartSimulation simStartSimulation;
extern ptrSimStopSimulation simStopSimulation;
extern ptrSimPauseSimulation simPauseSimulation;
extern ptrSimBroadcastMessage simBroadcastMessage;
extern ptrSimGetModuleName simGetModuleName;
extern ptrSimGetScriptSimulationParameter simGetScriptSimulationParameter;
extern ptrSimSetScriptSimulationParameter simSetScriptSimulationParameter;
extern ptrSimFloatingViewAdd simFloatingViewAdd;
extern ptrSimFloatingViewRemove simFloatingViewRemove;
extern ptrSimAdjustView simAdjustView;
extern ptrSimSetLastError simSetLastError;
extern ptrSimHandleGraph simHandleGraph;
extern ptrSimResetGraph simResetGraph;
extern ptrSimSetNavigationMode simSetNavigationMode;
extern ptrSimGetNavigationMode simGetNavigationMode;
extern ptrSimSetPage simSetPage;
extern ptrSimGetPage simGetPage;
extern ptrSimDisplayDialog simDisplayDialog;
extern ptrSimGetDialogResult simGetDialogResult;
extern ptrSimGetDialogInput simGetDialogInput;
extern ptrSimEndDialog simEndDialog;
extern ptrSimRegisterScriptCallbackFunction simRegisterScriptCallbackFunction;
extern ptrSimRegisterScriptVariable simRegisterScriptVariable;
extern ptrSimSetJointTargetVelocity simSetJointTargetVelocity;
extern ptrSimGetJointTargetVelocity simGetJointTargetVelocity;
extern ptrSimSetPathTargetNominalVelocity simSetPathTargetNominalVelocity;
extern ptrSimGetScriptRawBuffer simGetScriptRawBuffer;
extern ptrSimSetScriptRawBuffer simSetScriptRawBuffer;
extern ptrSimReleaseScriptRawBuffer simReleaseScriptRawBuffer;
extern ptrSimCopyPasteObjects simCopyPasteObjects;
extern ptrSimScaleSelectedObjects simScaleSelectedObjects;
extern ptrSimScaleObjects simScaleObjects;
extern ptrSimDeleteSelectedObjects simDeleteSelectedObjects;
extern ptrSimGetObjectUniqueIdentifier simGetObjectUniqueIdentifier;
extern ptrSimGetNameSuffix simGetNameSuffix;
extern ptrSimSendData simSendData;
extern ptrSimReceiveData simReceiveData;
extern ptrSimSetGraphUserData simSetGraphUserData;
extern ptrSimSetNameSuffix simSetNameSuffix;
extern ptrSimAddDrawingObject simAddDrawingObject;
extern ptrSimRemoveDrawingObject simRemoveDrawingObject;
extern ptrSimAddDrawingObjectItem simAddDrawingObjectItem;
extern ptrSimAddParticleObject simAddParticleObject;
extern ptrSimRemoveParticleObject simRemoveParticleObject;
extern ptrSimAddParticleObjectItem simAddParticleObjectItem;
extern ptrSimGetObjectSizeFactor simGetObjectSizeFactor;
extern ptrSimAnnounceSceneContentChange simAnnounceSceneContentChange;
extern ptrSimResetMilling simResetMilling;
extern ptrSimApplyMilling simApplyMilling;
extern ptrSimSetIntegerSignal simSetIntegerSignal;
extern ptrSimGetIntegerSignal simGetIntegerSignal;
extern ptrSimClearIntegerSignal simClearIntegerSignal;
extern ptrSimSetFloatSignal simSetFloatSignal;
extern ptrSimGetFloatSignal simGetFloatSignal;
extern ptrSimClearFloatSignal simClearFloatSignal;
extern ptrSimSetStringSignal simSetStringSignal;
extern ptrSimGetStringSignal simGetStringSignal;
extern ptrSimClearStringSignal simClearStringSignal;
extern ptrSimGetSignalName simGetSignalName;
extern ptrSimSetObjectProperty simSetObjectProperty;
extern ptrSimGetObjectProperty simGetObjectProperty;
extern ptrSimSetObjectSpecialProperty simSetObjectSpecialProperty;
extern ptrSimGetObjectSpecialProperty simGetObjectSpecialProperty;
extern ptrSimGetPositionOnPath simGetPositionOnPath;
extern ptrSimGetDataOnPath simGetDataOnPath;
extern ptrSimGetOrientationOnPath simGetOrientationOnPath;
extern ptrSimGetClosestPositionOnPath simGetClosestPositionOnPath;
extern ptrSimReadForceSensor simReadForceSensor;
extern ptrSimBreakForceSensor simBreakForceSensor;
extern ptrSimGetShapeVertex simGetShapeVertex;
extern ptrSimGetShapeTriangle simGetShapeTriangle;
extern ptrSimSetLightParameters simSetLightParameters;
extern ptrSimGetLightParameters simGetLightParameters;
extern ptrSimGetVelocity simGetVelocity;
extern ptrSimGetObjectVelocity simGetObjectVelocity;
extern ptrSimAddForceAndTorque simAddForceAndTorque;
extern ptrSimAddForce simAddForce;
extern ptrSimSetExplicitHandling simSetExplicitHandling;
extern ptrSimGetExplicitHandling simGetExplicitHandling;
extern ptrSimGetLinkDummy simGetLinkDummy;
extern ptrSimSetLinkDummy simSetLinkDummy;
extern ptrSimSetModelProperty simSetModelProperty;
extern ptrSimGetModelProperty simGetModelProperty;
extern ptrSimSetShapeColor simSetShapeColor;
extern ptrSimGetShapeColor simGetShapeColor;
extern ptrSimResetDynamicObject simResetDynamicObject;
extern ptrSimSetJointMode simSetJointMode;
extern ptrSimGetJointMode simGetJointMode;
extern ptrSimSerialOpen simSerialOpen;
extern ptrSimSerialClose simSerialClose;
extern ptrSimSerialSend simSerialSend;
extern ptrSimSerialRead simSerialRead;
extern ptrSimSerialCheck simSerialCheck;
extern ptrSimGetContactInfo simGetContactInfo;
extern ptrSimSetThreadIsFree simSetThreadIsFree;
extern ptrSimTubeOpen simTubeOpen;
extern ptrSimTubeClose simTubeClose;
extern ptrSimTubeWrite simTubeWrite;
extern ptrSimTubeRead simTubeRead;
extern ptrSimTubeStatus simTubeStatus;
extern ptrSimAuxiliaryConsoleOpen simAuxiliaryConsoleOpen;
extern ptrSimAuxiliaryConsoleClose simAuxiliaryConsoleClose;
extern ptrSimAuxiliaryConsoleShow simAuxiliaryConsoleShow;
extern ptrSimAuxiliaryConsolePrint simAuxiliaryConsolePrint;
extern ptrSimImportShape simImportShape;
extern ptrSimImportMesh simImportMesh;
extern ptrSimExportMesh simExportMesh;
extern ptrSimCreateMeshShape simCreateMeshShape;
extern ptrSimCreatePureShape simCreatePureShape;
extern ptrSimCreateHeightfieldShape simCreateHeightfieldShape;
extern ptrSimGetShapeMesh simGetShapeMesh;
extern ptrSimAddBanner simAddBanner;
extern ptrSimRemoveBanner simRemoveBanner;
extern ptrSimCreateJoint simCreateJoint;
extern ptrSimCreateDummy simCreateDummy;
extern ptrSimCreateForceSensor simCreateForceSensor;
extern ptrSimCreateVisionSensor simCreateVisionSensor;
extern ptrSimCreateProximitySensor simCreateProximitySensor;
extern ptrSimCreatePath simCreatePath;
extern ptrSimInsertPathCtrlPoints simInsertPathCtrlPoints;
extern ptrSimCutPathCtrlPoints simCutPathCtrlPoints;
extern ptrSimGetObjectIntParameter simGetObjectIntParameter;
extern ptrSimSetObjectIntParameter simSetObjectIntParameter;
extern ptrSimGetObjectInt32Parameter simGetObjectInt32Parameter;
extern ptrSimSetObjectInt32Parameter simSetObjectInt32Parameter;
extern ptrSimGetObjectFloatParameter simGetObjectFloatParameter;
extern ptrSimSetObjectFloatParameter simSetObjectFloatParameter;
extern ptrSimGetObjectStringParameter simGetObjectStringParameter;
extern ptrSimSetObjectStringParameter simSetObjectStringParameter;
extern ptrSimSetSimulationPassesPerRenderingPass simSetSimulationPassesPerRenderingPass;
extern ptrSimGetRotationAxis simGetRotationAxis;
extern ptrSimRotateAroundAxis simRotateAroundAxis;
extern ptrSimGetJointForce simGetJointForce;
extern ptrSimSetArrayParameter simSetArrayParameter;
extern ptrSimGetArrayParameter simGetArrayParameter;
extern ptrSimSetIkGroupProperties simSetIkGroupProperties;
extern ptrSimSetIkElementProperties simSetIkElementProperties;
extern ptrSimCameraFitToView simCameraFitToView;
extern ptrSimPersistentDataWrite simPersistentDataWrite;
extern ptrSimPersistentDataRead simPersistentDataRead;
extern ptrSimIsHandleValid simIsHandleValid;
extern ptrSimHandleVisionSensor simHandleVisionSensor;
extern ptrSimReadVisionSensor simReadVisionSensor;
extern ptrSimResetVisionSensor simResetVisionSensor;
extern ptrSimCheckVisionSensor simCheckVisionSensor;
extern ptrSimCheckVisionSensorEx simCheckVisionSensorEx;
extern ptrSimGetVisionSensorResolution simGetVisionSensorResolution;
extern ptrSimGetVisionSensorImage simGetVisionSensorImage;
extern ptrSimGetVisionSensorCharImage simGetVisionSensorCharImage;
extern ptrSimSetVisionSensorImage simSetVisionSensorImage;
extern ptrSimSetVisionSensorCharImage simSetVisionSensorCharImage;
extern ptrSimGetVisionSensorDepthBuffer simGetVisionSensorDepthBuffer;
extern ptrSimGetObjectQuaternion simGetObjectQuaternion;
extern ptrSimSetObjectQuaternion simSetObjectQuaternion;
extern ptrSimRMLPosition simRMLPosition;
extern ptrSimRMLVelocity simRMLVelocity;
extern ptrSimRMLPos simRMLPos;
extern ptrSimRMLVel simRMLVel;
extern ptrSimRMLStep simRMLStep;
extern ptrSimRMLRemove simRMLRemove;
extern ptrSimBuildMatrixQ simBuildMatrixQ;
extern ptrSimGetQuaternionFromMatrix simGetQuaternionFromMatrix;
extern ptrSimFileDialog simFileDialog;
extern ptrSimMsgBox simMsgBox;
extern ptrSimSetShapeMassAndInertia simSetShapeMassAndInertia;
extern ptrSimGetShapeMassAndInertia simGetShapeMassAndInertia;
extern ptrSimGroupShapes simGroupShapes;
extern ptrSimUngroupShape simUngroupShape;
extern ptrSimConvexDecompose simConvexDecompose;
extern ptrSimGetIkGroupMatrix simGetIkGroupMatrix;
extern ptrSimAddGhost simAddGhost;
extern ptrSimModifyGhost simModifyGhost;
extern ptrSimQuitSimulator simQuitSimulator;
extern ptrSimGetThreadId simGetThreadId;
extern ptrSimLockResources simLockResources;
extern ptrSimUnlockResources simUnlockResources;
extern ptrSimEnableEventCallback simEnableEventCallback;
extern ptrSimSetShapeMaterial simSetShapeMaterial;
extern ptrSimGetTextureId simGetTextureId;
extern ptrSimReadTexture simReadTexture;
extern ptrSimWriteTexture simWriteTexture;
extern ptrSimCreateTexture simCreateTexture;
extern ptrSimWriteCustomDataBlock simWriteCustomDataBlock;
extern ptrSimReadCustomDataBlock simReadCustomDataBlock;
extern ptrSimReadCustomDataBlockTags simReadCustomDataBlockTags;
extern ptrSimAddPointCloud simAddPointCloud;
extern ptrSimModifyPointCloud simModifyPointCloud;
extern ptrSimGetShapeGeomInfo simGetShapeGeomInfo;
extern ptrSimGetObjectsInTree simGetObjectsInTree;
extern ptrSimSetObjectSizeValues simSetObjectSizeValues;
extern ptrSimGetObjectSizeValues simGetObjectSizeValues;
extern ptrSimScaleObject simScaleObject;
extern ptrSimSetShapeTexture simSetShapeTexture;
extern ptrSimGetShapeTextureId simGetShapeTextureId;
extern ptrSimGetCollectionObjects simGetCollectionObjects;
extern ptrSimHandleCustomizationScripts simHandleCustomizationScripts;
extern ptrSimSetScriptAttribute simSetScriptAttribute;
extern ptrSimGetScriptAttribute simGetScriptAttribute;
extern ptrSimReorientShapeBoundingBox simReorientShapeBoundingBox;
extern ptrSimSwitchThread simSwitchThread;
extern ptrSimCreateIkGroup simCreateIkGroup;
extern ptrSimRemoveIkGroup simRemoveIkGroup;
extern ptrSimCreateIkElement simCreateIkElement;
extern ptrSimCreateCollection simCreateCollection;
extern ptrSimAddObjectToCollection simAddObjectToCollection;
extern ptrSimSaveImage simSaveImage;
extern ptrSimLoadImage simLoadImage;
extern ptrSimGetScaledImage simGetScaledImage;
extern ptrSimTransformImage simTransformImage;
extern ptrSimGetQHull simGetQHull;
extern ptrSimGetDecimatedMesh simGetDecimatedMesh;
extern ptrSimExportIk simExportIk;
extern ptrSimCallScriptFunction simCallScriptFunction;
extern ptrSimCallScriptFunctionEx simCallScriptFunctionEx;
extern ptrSimComputeJacobian simComputeJacobian;
extern ptrSimGetConfigForTipPose simGetConfigForTipPose;
extern ptrSimGenerateIkPath simGenerateIkPath;
extern ptrSimGetExtensionString simGetExtensionString;
extern ptrSimComputeMassAndInertia simComputeMassAndInertia;
extern ptrSimCreateStack simCreateStack;
extern ptrSimReleaseStack simReleaseStack;
extern ptrSimCopyStack simCopyStack;
extern ptrSimPushNullOntoStack simPushNullOntoStack;
extern ptrSimPushBoolOntoStack simPushBoolOntoStack;
extern ptrSimPushInt32OntoStack simPushInt32OntoStack;
extern ptrSimPushFloatOntoStack simPushFloatOntoStack;
extern ptrSimPushDoubleOntoStack simPushDoubleOntoStack;
extern ptrSimPushStringOntoStack simPushStringOntoStack;
extern ptrSimPushUInt8TableOntoStack simPushUInt8TableOntoStack;
extern ptrSimPushInt32TableOntoStack simPushInt32TableOntoStack;
extern ptrSimPushFloatTableOntoStack simPushFloatTableOntoStack;
extern ptrSimPushDoubleTableOntoStack simPushDoubleTableOntoStack;
extern ptrSimPushTableOntoStack simPushTableOntoStack;
extern ptrSimInsertDataIntoStackTable simInsertDataIntoStackTable;
extern ptrSimGetStackSize simGetStackSize;
extern ptrSimPopStackItem simPopStackItem;
extern ptrSimMoveStackItemToTop simMoveStackItemToTop;
extern ptrSimIsStackValueNull simIsStackValueNull;
extern ptrSimGetStackBoolValue simGetStackBoolValue;
extern ptrSimGetStackInt32Value simGetStackInt32Value;
extern ptrSimGetStackFloatValue simGetStackFloatValue;
extern ptrSimGetStackDoubleValue simGetStackDoubleValue;
extern ptrSimGetStackStringValue simGetStackStringValue;
extern ptrSimGetStackTableInfo simGetStackTableInfo;
extern ptrSimGetStackUInt8Table simGetStackUInt8Table;
extern ptrSimGetStackInt32Table simGetStackInt32Table;
extern ptrSimGetStackFloatTable simGetStackFloatTable;
extern ptrSimGetStackDoubleTable simGetStackDoubleTable;
extern ptrSimUnfoldStackTable simUnfoldStackTable;
extern ptrSimDebugStack simDebugStack;
extern ptrSimSetScriptVariable simSetScriptVariable;
extern ptrSimGetEngineFloatParameter simGetEngineFloatParameter;
extern ptrSimGetEngineInt32Parameter simGetEngineInt32Parameter;
extern ptrSimGetEngineBoolParameter simGetEngineBoolParameter;
extern ptrSimSetEngineFloatParameter simSetEngineFloatParameter;
extern ptrSimSetEngineInt32Parameter simSetEngineInt32Parameter;
extern ptrSimSetEngineBoolParameter simSetEngineBoolParameter;
extern ptrSimCreateOctree simCreateOctree;
extern ptrSimCreatePointCloud simCreatePointCloud;
extern ptrSimSetPointCloudOptions simSetPointCloudOptions;
extern ptrSimGetPointCloudOptions simGetPointCloudOptions;
extern ptrSimInsertVoxelsIntoOctree simInsertVoxelsIntoOctree;
extern ptrSimRemoveVoxelsFromOctree simRemoveVoxelsFromOctree;
extern ptrSimInsertPointsIntoPointCloud simInsertPointsIntoPointCloud;
extern ptrSimRemovePointsFromPointCloud simRemovePointsFromPointCloud;
extern ptrSimIntersectPointsWithPointCloud simIntersectPointsWithPointCloud;
extern ptrSimGetOctreeVoxels simGetOctreeVoxels;
extern ptrSimGetPointCloudPoints simGetPointCloudPoints;
extern ptrSimInsertObjectIntoOctree simInsertObjectIntoOctree;
extern ptrSimSubtractObjectFromOctree simSubtractObjectFromOctree;
extern ptrSimInsertObjectIntoPointCloud simInsertObjectIntoPointCloud;
extern ptrSimSubtractObjectFromPointCloud simSubtractObjectFromPointCloud;
extern ptrSimCheckOctreePointOccupancy simCheckOctreePointOccupancy;
extern ptrSimOpenTextEditor simOpenTextEditor;
extern ptrSimPackTable simPackTable;
extern ptrSimUnpackTable simUnpackTable;
extern ptrSimSetVisionSensorFilter simSetVisionSensorFilter;
extern ptrSimGetVisionSensorFilter simGetVisionSensorFilter;
extern ptrSimSetReferencedHandles simSetReferencedHandles;
extern ptrSimGetReferencedHandles simGetReferencedHandles;
extern ptrSimGetShapeViz simGetShapeViz;
extern ptrSimExecuteScriptString simExecuteScriptString;
extern ptrSimGetApiFunc simGetApiFunc;
extern ptrSimGetApiInfo simGetApiInfo;
extern ptrSimSetModuleInfo simSetModuleInfo;
extern ptrSimGetModuleInfo simGetModuleInfo;


extern ptr_simGetContactCallbackCount _simGetContactCallbackCount;
extern ptr_simGetContactCallback _simGetContactCallback;
extern ptr_simSetDynamicSimulationIconCode _simSetDynamicSimulationIconCode;
extern ptr_simSetDynamicObjectFlagForVisualization _simSetDynamicObjectFlagForVisualization;
extern ptr_simGetObjectListSize _simGetObjectListSize;
extern ptr_simGetObjectFromIndex _simGetObjectFromIndex;
extern ptr_simGetObjectID _simGetObjectID;
extern ptr_simGetObjectType _simGetObjectType;
extern ptr_simGetObjectChildren _simGetObjectChildren;
extern ptr_simGetGeomProxyFromShape _simGetGeomProxyFromShape;
extern ptr_simGetParentObject _simGetParentObject;
extern ptr_simGetObject _simGetObject;
extern ptr_simGetIkGroupObject _simGetIkGroupObject;
extern ptr_simMpHandleIkGroupObject _simMpHandleIkGroupObject;
extern ptr_simGetObjectLocalTransformation _simGetObjectLocalTransformation;
extern ptr_simSetObjectLocalTransformation _simSetObjectLocalTransformation;
extern ptr_simSetObjectCumulativeTransformation _simSetObjectCumulativeTransformation;
extern ptr_simGetObjectCumulativeTransformation _simGetObjectCumulativeTransformation;
extern ptr_simIsShapeDynamicallyStatic _simIsShapeDynamicallyStatic;
extern ptr_simGetTreeDynamicProperty _simGetTreeDynamicProperty;
extern ptr_simGetDummyLinkType _simGetDummyLinkType;
extern ptr_simGetJointMode _simGetJointMode;
extern ptr_simIsJointInHybridOperation _simIsJointInHybridOperation;
extern ptr_simDisableDynamicTreeForManipulation _simDisableDynamicTreeForManipulation;
extern ptr_simIsShapeDynamicallyRespondable _simIsShapeDynamicallyRespondable;
extern ptr_simGetDynamicCollisionMask _simGetDynamicCollisionMask;
extern ptr_simGetLastParentForLocalGlobalCollidable _simGetLastParentForLocalGlobalCollidable;
extern ptr_simSetShapeIsStaticAndNotRespondableButDynamicTag _simSetShapeIsStaticAndNotRespondableButDynamicTag;
extern ptr_simGetShapeIsStaticAndNotRespondableButDynamicTag _simGetShapeIsStaticAndNotRespondableButDynamicTag;
extern ptr_simSetJointPosition _simSetJointPosition;
extern ptr_simGetJointPosition _simGetJointPosition;
extern ptr_simSetDynamicMotorPositionControlTargetPosition _simSetDynamicMotorPositionControlTargetPosition;
extern ptr_simGetInitialDynamicVelocity _simGetInitialDynamicVelocity;
extern ptr_simSetInitialDynamicVelocity _simSetInitialDynamicVelocity;
extern ptr_simGetInitialDynamicAngVelocity _simGetInitialDynamicAngVelocity;
extern ptr_simSetInitialDynamicAngVelocity _simSetInitialDynamicAngVelocity;
extern ptr_simGetStartSleeping _simGetStartSleeping;
extern ptr_simGetWasPutToSleepOnce _simGetWasPutToSleepOnce;
extern ptr_simGetDynamicsFullRefreshFlag _simGetDynamicsFullRefreshFlag;
extern ptr_simSetDynamicsFullRefreshFlag _simSetDynamicsFullRefreshFlag;
extern ptr_simSetGeomProxyDynamicsFullRefreshFlag _simSetGeomProxyDynamicsFullRefreshFlag;
extern ptr_simGetGeomProxyDynamicsFullRefreshFlag _simGetGeomProxyDynamicsFullRefreshFlag;
extern ptr_simGetParentFollowsDynamic _simGetParentFollowsDynamic;
extern ptr_simSetShapeDynamicVelocity _simSetShapeDynamicVelocity;
extern ptr_simGetAdditionalForceAndTorque _simGetAdditionalForceAndTorque;
extern ptr_simClearAdditionalForceAndTorque _simClearAdditionalForceAndTorque;
extern ptr_simGetJointPositionInterval _simGetJointPositionInterval;
extern ptr_simGetJointType _simGetJointType;
extern ptr_simIsForceSensorBroken _simIsForceSensorBroken;
extern ptr_simGetDynamicForceSensorLocalTransformationPart2 _simGetDynamicForceSensorLocalTransformationPart2;
extern ptr_simIsDynamicMotorEnabled _simIsDynamicMotorEnabled;
extern ptr_simIsDynamicMotorPositionCtrlEnabled _simIsDynamicMotorPositionCtrlEnabled;
extern ptr_simIsDynamicMotorTorqueModulationEnabled _simIsDynamicMotorTorqueModulationEnabled;
extern ptr_simGetMotorPid _simGetMotorPid;
extern ptr_simGetDynamicMotorTargetPosition _simGetDynamicMotorTargetPosition;
extern ptr_simGetDynamicMotorTargetVelocity _simGetDynamicMotorTargetVelocity;
extern ptr_simGetDynamicMotorMaxForce _simGetDynamicMotorMaxForce;
extern ptr_simGetDynamicMotorUpperLimitVelocity _simGetDynamicMotorUpperLimitVelocity;
extern ptr_simSetDynamicMotorReflectedPositionFromDynamicEngine _simSetDynamicMotorReflectedPositionFromDynamicEngine;
extern ptr_simSetJointSphericalTransformation _simSetJointSphericalTransformation;
extern ptr_simAddForceSensorCumulativeForcesAndTorques _simAddForceSensorCumulativeForcesAndTorques;
extern ptr_simAddJointCumulativeForcesOrTorques _simAddJointCumulativeForcesOrTorques;
extern ptr_simSetDynamicJointLocalTransformationPart2 _simSetDynamicJointLocalTransformationPart2;
extern ptr_simSetDynamicForceSensorLocalTransformationPart2 _simSetDynamicForceSensorLocalTransformationPart2;
extern ptr_simSetDynamicJointLocalTransformationPart2IsValid _simSetDynamicJointLocalTransformationPart2IsValid;
extern ptr_simSetDynamicForceSensorLocalTransformationPart2IsValid _simSetDynamicForceSensorLocalTransformationPart2IsValid;
extern ptr_simGetGeomWrapFromGeomProxy _simGetGeomWrapFromGeomProxy;
extern ptr_simGetLocalInertiaFrame _simGetLocalInertiaFrame;
extern ptr_simGetPurePrimitiveType _simGetPurePrimitiveType;
extern ptr_simIsGeomWrapGeometric _simIsGeomWrapGeometric;
extern ptr_simIsGeomWrapConvex _simIsGeomWrapConvex;
extern ptr_simGetGeometricCount _simGetGeometricCount;
extern ptr_simGetAllGeometrics _simGetAllGeometrics;
extern ptr_simGetPurePrimitiveSizes _simGetPurePrimitiveSizes;
extern ptr_simMakeDynamicAnnouncement _simMakeDynamicAnnouncement;
extern ptr_simGetVerticesLocalFrame _simGetVerticesLocalFrame;
extern ptr_simGetHeightfieldData _simGetHeightfieldData;
extern ptr_simGetCumulativeMeshes _simGetCumulativeMeshes;
extern ptr_simGetMass _simGetMass;
extern ptr_simGetPrincipalMomentOfInertia _simGetPrincipalMomentOfInertia;
extern ptr_simGetGravity _simGetGravity;
extern ptr_simGetTimeDiffInMs _simGetTimeDiffInMs;
extern ptr_simDoEntitiesCollide _simDoEntitiesCollide;
extern ptr_simGetDistanceBetweenEntitiesIfSmaller _simGetDistanceBetweenEntitiesIfSmaller;
extern ptr_simHandleJointControl _simHandleJointControl;
extern ptr_simHandleCustomContact _simHandleCustomContact;
extern ptr_simGetPureHollowScaling _simGetPureHollowScaling;
extern ptr_simGetJointCallbackCallOrder _simGetJointCallbackCallOrder;
extern ptr_simDynCallback _simDynCallback;


// Deprecated begin
typedef simInt (__cdecl *ptrSimGetMaterialId)(const simChar *materialName);

typedef simInt (__cdecl *ptrSimGetShapeMaterial)(simInt shapeHandle);

typedef simInt (__cdecl *ptrSimHandleVarious)();

typedef simInt (__cdecl *ptrSimSerialPortOpen)(simInt portNumber, simInt baudRate, simVoid *reserved1, simVoid *reserved2);

typedef simInt (__cdecl *ptrSimSerialPortClose)(simInt portNumber);

typedef simInt (__cdecl *ptrSimSerialPortSend)(simInt portNumber, const simChar *data, simInt dataLength);

typedef simInt (__cdecl *ptrSimSerialPortRead)(simInt portNumber, simChar *buffer, simInt dataLengthToRead);

typedef simInt (__cdecl *ptrSimJointGetForce)(simInt jointHandle, simFloat *forceOrTorque);

typedef simInt (__cdecl *ptrSimGetPathPlanningHandle)(const simChar *pathPlanningObjectName);

typedef simInt (__cdecl *ptrSimGetMotionPlanningHandle)(const simChar *motionPlanningObjectName);

typedef simInt (__cdecl *ptrSimGetMpConfigForTipPose)(simInt motionPlanningObjectHandle, simInt options, simFloat closeNodesDistance, simInt trialCount, const simFloat *tipPose, simInt maxTimeInMs, simFloat *outputJointPositions, const simFloat *referenceConfigs, simInt referenceConfigCount, const simFloat *jointWeights, const simInt *jointBehaviour, simInt correctionPasses);

typedef simFloat *(__cdecl *ptrSimFindMpPath)(simInt motionPlanningObjectHandle, const simFloat *startConfig, const simFloat *goalConfig, simInt options, simFloat stepSize, simInt *outputConfigsCnt, simInt maxTimeInMs, simFloat *reserved, const simInt *auxIntParams, const simFloat *auxFloatParams);

typedef simFloat *(__cdecl *ptrSimSimplifyMpPath)(simInt motionPlanningObjectHandle, const simFloat *pathBuffer, simInt configCnt, simInt options, simFloat stepSize, simInt increment, simInt *outputConfigsCnt, simInt maxTimeInMs, simFloat *reserved, const simInt *auxIntParams, const simFloat *auxFloatParams);

typedef simFloat *(__cdecl *ptrSimFindIkPath)(simInt motionPlanningObjectHandle, const simFloat *startConfig, const simFloat *goalPose, simInt options, simFloat stepSize, simInt *outputConfigsCnt, simFloat *reserved, const simInt *auxIntParams, const simFloat *auxFloatParams);

typedef simFloat *(__cdecl *ptrSimGetMpConfigTransition)(simInt motionPlanningObjectHandle, const simFloat *startConfig, const simFloat *goalConfig, simInt options, const simInt *select, simFloat calcStepSize, simFloat maxOutStepSize, simInt wayPointCnt, const simFloat *wayPoints, simInt *outputConfigsCnt, const simInt *auxIntParams, const simFloat *auxFloatParams);

typedef simInt (__cdecl *ptrSimCreateMotionPlanning)(simInt jointCnt, const simInt *jointHandles, const simInt *jointRangeSubdivisions, const simFloat *jointMetricWeights, simInt options, const simInt *intParams, const simFloat *floatParams, const simVoid *reserved);

typedef simInt (__cdecl *ptrSimRemoveMotionPlanning)(simInt motionPlanningHandle);

typedef simInt (__cdecl *ptrSimSearchPath)(simInt pathPlanningObjectHandle, simFloat maximumSearchTime);

typedef simInt (__cdecl *ptrSimInitializePathSearch)(simInt pathPlanningObjectHandle, simFloat maximumSearchTime, simFloat searchTimeStep);

typedef simInt (__cdecl *ptrSimPerformPathSearchStep)(simInt temporaryPathSearchObject, simBool abortSearch);

typedef simInt (__cdecl *ptrSimLockInterface)(simBool locked);

typedef simInt (__cdecl *ptrSimCopyPasteSelectedObjects)();

typedef simInt (__cdecl *ptrSimResetPath)(simInt pathHandle);

typedef simInt (__cdecl *ptrSimHandlePath)(simInt pathHandle, simFloat deltaTime);

typedef simInt (__cdecl *ptrSimResetJoint)(simInt jointHandle);

typedef simInt (__cdecl *ptrSimHandleJoint)(simInt jointHandle, simFloat deltaTime);

typedef simInt (__cdecl *ptrSimAppendScriptArrayEntry)(const simChar *reservedSetToNull, simInt scriptHandleOrType, const simChar *arrayNameAtScriptName, const simChar *keyName, const simChar *data, const simInt *what);

typedef simInt (__cdecl *ptrSimClearScriptVariable)(const simChar *reservedSetToNull, simInt scriptHandleOrType, const simChar *variableNameAtScriptName);

typedef simVoid (__cdecl *ptr_simGetJointOdeParameters)(const simVoid *joint, simFloat *stopERP, simFloat *stopCFM, simFloat *bounce, simFloat *fudge, simFloat *normalCFM);

typedef simVoid (__cdecl *ptr_simGetJointBulletParameters)(const simVoid *joint, simFloat *stopERP, simFloat *stopCFM, simFloat *normalCFM);

typedef simVoid (__cdecl *ptr_simGetOdeMaxContactFrictionCFMandERP)(const simVoid *geomInfo, simInt *maxContacts, simFloat *friction, simFloat *cfm, simFloat *erp);

typedef simBool (__cdecl *ptr_simGetBulletCollisionMargin)(const simVoid *geomInfo, simFloat *margin, simInt *otherProp);

typedef simBool (__cdecl *ptr_simGetBulletStickyContact)(const simVoid *geomInfo);

typedef simFloat (__cdecl *ptr_simGetBulletRestitution)(const simVoid *geomInfo);

typedef simVoid (__cdecl *ptr_simGetVortexParameters)(const simVoid *object, simInt version, simFloat *floatParams, simInt *intParams);

typedef simVoid (__cdecl *ptr_simGetNewtonParameters)(const simVoid *object, simInt *version, simFloat *floatParams, simInt *intParams);

typedef simVoid (__cdecl *ptr_simGetDamping)(const simVoid *geomInfo, simFloat *linDamping, simFloat *angDamping);

typedef simFloat (__cdecl *ptr_simGetFriction)(const simVoid *geomInfo);

typedef simInt (__cdecl *ptrSimAddSceneCustomData)(simInt header, const simChar *data, simInt dataLength);

typedef simInt (__cdecl *ptrSimGetSceneCustomDataLength)(simInt header);

typedef simInt (__cdecl *ptrSimGetSceneCustomData)(simInt header, simChar *data);

typedef simInt (__cdecl *ptrSimAddObjectCustomData)(simInt objectHandle, simInt header, const simChar *data, simInt dataLength);

typedef simInt (__cdecl *ptrSimGetObjectCustomDataLength)(simInt objectHandle, simInt header);

typedef simInt (__cdecl *ptrSimGetObjectCustomData)(simInt objectHandle, simInt header, simChar *data);

typedef simInt (__cdecl *ptrSimCreateUI)(const simChar *uiName, simInt menuAttributes, const simInt *clientSize, const simInt *cellSize, simInt *buttonHandles);

typedef simInt (__cdecl *ptrSimCreateUIButton)(simInt uiHandle, const simInt *position, const simInt *size, simInt buttonProperty);

typedef simInt (__cdecl *ptrSimGetUIHandle)(const simChar *uiName);

typedef simInt (__cdecl *ptrSimGetUIProperty)(simInt uiHandle);

typedef simInt (__cdecl *ptrSimGetUIEventButton)(simInt uiHandle, simInt *auxiliaryValues);

typedef simInt (__cdecl *ptrSimSetUIProperty)(simInt uiHandle, simInt elementProperty);

typedef simInt (__cdecl *ptrSimGetUIButtonProperty)(simInt uiHandle, simInt buttonHandle);

typedef simInt (__cdecl *ptrSimSetUIButtonProperty)(simInt uiHandle, simInt buttonHandle, simInt buttonProperty);

typedef simInt (__cdecl *ptrSimGetUIButtonSize)(simInt uiHandle, simInt buttonHandle, simInt *size);

typedef simInt (__cdecl *ptrSimSetUIButtonLabel)(simInt uiHandle, simInt buttonHandle, const simChar *upStateLabel, const simChar *downStateLabel);

typedef simChar *(__cdecl *ptrSimGetUIButtonLabel)(simInt uiHandle, simInt buttonHandle);

typedef simInt (__cdecl *ptrSimSetUISlider)(simInt uiHandle, simInt buttonHandle, simInt position);

typedef simInt (__cdecl *ptrSimGetUISlider)(simInt uiHandle, simInt buttonHandle);

typedef simInt (__cdecl *ptrSimSetUIButtonColor)(simInt uiHandle, simInt buttonHandle, const simFloat *upStateColor, const simFloat *downStateColor, const simFloat *labelColor);

typedef simInt (__cdecl *ptrSimSetUIButtonTexture)(simInt uiHandle, simInt buttonHandle, const simInt *size, const simChar *textureData);

typedef simInt (__cdecl *ptrSimCreateUIButtonArray)(simInt uiHandle, simInt buttonHandle);

typedef simInt (__cdecl *ptrSimSetUIButtonArrayColor)(simInt uiHandle, simInt buttonHandle, const simInt *position, const simFloat *color);

typedef simInt (__cdecl *ptrSimDeleteUIButtonArray)(simInt uiHandle, simInt buttonHandle);

typedef simInt (__cdecl *ptrSimRemoveUI)(simInt uiHandle);

typedef simInt (__cdecl *ptrSimSetUIPosition)(simInt uiHandle, const simInt *position);

typedef simInt (__cdecl *ptrSimGetUIPosition)(simInt uiHandle, simInt *position);

typedef simInt (__cdecl *ptrSimLoadUI)(const simChar *filename, int maxCount, int *uiHandles);

typedef simInt (__cdecl *ptrSimSaveUI)(int count, const int *uiHandles, const simChar *filename);

typedef simInt (__cdecl *ptrSimHandleGeneralCallbackScript)(simInt callbackId, simInt callbackTag, simVoid *additionalData);

typedef simInt (__cdecl *ptrSimRegisterCustomLuaFunction)(const simChar *funcName, const simChar *callTips, const simInt *inputArgumentTypes, simVoid(*callBack)(struct SLuaCallBack *p));

typedef simInt (__cdecl *ptrSimRegisterCustomLuaVariable)(const simChar *varName, const simChar *varValue);

typedef simInt (__cdecl *ptrSimRegisterContactCallback)(simInt(*callBack)(simInt, simInt, simInt, simInt *, simFloat *));

extern ptrSimGetMaterialId simGetMaterialId;
extern ptrSimGetShapeMaterial simGetShapeMaterial;
extern ptrSimHandleVarious simHandleVarious;
extern ptrSimSerialPortOpen simSerialPortOpen;
extern ptrSimSerialPortClose simSerialPortClose;
extern ptrSimSerialPortSend simSerialPortSend;
extern ptrSimSerialPortRead simSerialPortRead;
extern ptrSimJointGetForce simJointGetForce;
extern ptrSimGetPathPlanningHandle simGetPathPlanningHandle;
extern ptrSimGetMotionPlanningHandle simGetMotionPlanningHandle;
extern ptrSimGetMpConfigForTipPose simGetMpConfigForTipPose;
extern ptrSimFindMpPath simFindMpPath;
extern ptrSimSimplifyMpPath simSimplifyMpPath;
extern ptrSimFindIkPath simFindIkPath;
extern ptrSimGetMpConfigTransition simGetMpConfigTransition;
extern ptrSimCreateMotionPlanning simCreateMotionPlanning;
extern ptrSimRemoveMotionPlanning simRemoveMotionPlanning;
extern ptrSimSearchPath simSearchPath;
extern ptrSimInitializePathSearch simInitializePathSearch;
extern ptrSimPerformPathSearchStep simPerformPathSearchStep;
extern ptrSimLockInterface simLockInterface;
extern ptrSimCopyPasteSelectedObjects simCopyPasteSelectedObjects;
extern ptrSimResetPath simResetPath;
extern ptrSimHandlePath simHandlePath;
extern ptrSimResetJoint simResetJoint;
extern ptrSimHandleJoint simHandleJoint;
extern ptrSimAppendScriptArrayEntry simAppendScriptArrayEntry;
extern ptrSimClearScriptVariable simClearScriptVariable;
extern ptr_simGetJointOdeParameters _simGetJointOdeParameters;
extern ptr_simGetJointBulletParameters _simGetJointBulletParameters;
extern ptr_simGetOdeMaxContactFrictionCFMandERP _simGetOdeMaxContactFrictionCFMandERP;
extern ptr_simGetBulletCollisionMargin _simGetBulletCollisionMargin;
extern ptr_simGetBulletStickyContact _simGetBulletStickyContact;
extern ptr_simGetBulletRestitution _simGetBulletRestitution;
extern ptr_simGetVortexParameters _simGetVortexParameters;
extern ptr_simGetNewtonParameters _simGetNewtonParameters;
extern ptr_simGetDamping _simGetDamping;
extern ptr_simGetFriction _simGetFriction;
extern ptrSimAddSceneCustomData simAddSceneCustomData;
extern ptrSimGetSceneCustomDataLength simGetSceneCustomDataLength;
extern ptrSimGetSceneCustomData simGetSceneCustomData;
extern ptrSimAddObjectCustomData simAddObjectCustomData;
extern ptrSimGetObjectCustomDataLength simGetObjectCustomDataLength;
extern ptrSimGetObjectCustomData simGetObjectCustomData;
extern ptrSimCreateUI simCreateUI;
extern ptrSimCreateUIButton simCreateUIButton;
extern ptrSimGetUIHandle simGetUIHandle;
extern ptrSimGetUIProperty simGetUIProperty;
extern ptrSimGetUIEventButton simGetUIEventButton;
extern ptrSimSetUIProperty simSetUIProperty;
extern ptrSimGetUIButtonProperty simGetUIButtonProperty;
extern ptrSimSetUIButtonProperty simSetUIButtonProperty;
extern ptrSimGetUIButtonSize simGetUIButtonSize;
extern ptrSimSetUIButtonLabel simSetUIButtonLabel;
extern ptrSimGetUIButtonLabel simGetUIButtonLabel;
extern ptrSimSetUISlider simSetUISlider;
extern ptrSimGetUISlider simGetUISlider;
extern ptrSimSetUIButtonColor simSetUIButtonColor;
extern ptrSimSetUIButtonTexture simSetUIButtonTexture;
extern ptrSimCreateUIButtonArray simCreateUIButtonArray;
extern ptrSimSetUIButtonArrayColor simSetUIButtonArrayColor;
extern ptrSimDeleteUIButtonArray simDeleteUIButtonArray;
extern ptrSimRemoveUI simRemoveUI;
extern ptrSimSetUIPosition simSetUIPosition;
extern ptrSimGetUIPosition simGetUIPosition;
extern ptrSimLoadUI simLoadUI;
extern ptrSimSaveUI simSaveUI;
extern ptrSimHandleGeneralCallbackScript simHandleGeneralCallbackScript;
extern ptrSimRegisterCustomLuaFunction simRegisterCustomLuaFunction;
extern ptrSimRegisterCustomLuaVariable simRegisterCustomLuaVariable;
extern ptrSimRegisterContactCallback simRegisterContactCallback;
// Deprecated end

#endif // !defined(V_REPLIB_INCLUDED_)
