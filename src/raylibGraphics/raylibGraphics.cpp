#include "raylibGraphics.hpp"
#include "iostream"
#include "rlgl.h"

RaylibGraphics::RaylibGraphics(SimThread* _simThread) 
    : simThread(_simThread)
{
    // Initialize camera
    camera.position = (Vector3){ 15.0f, 5.0f, 15.0f };
    camera.target = (Vector3){ 0.0f, 5.0f, 0.0f };
    camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    camera.fovy = 60.0f;
    camera.projection = CAMERA_PERSPECTIVE;
}

// Muunna drone-koordinaatisto (Z ylös) → Raylib-koordinaatisto (Y ylös)
Vector3 toRaylib(const Vector3f& v) {
    return {v.x(), v.z(), v.y()}; // drone X → raylib X, drone Z → raylib Y (ylös), drone Y → raylib Z
}

std::vector<Vector3> vectorToRaylib(std::vector<Vector3f> vector){
    std::vector<Vector3> result;
    for (const auto& v : vector) {
        result.push_back(toRaylib(v));
    }
    return result;
}

// Helper function: draw arrow from origin to origin + direction
void drawArrow(Vector3 origin, Vector3 direction, Color color, float scale = 1.0f) {
    Vector3 end = Vector3Add(origin, Vector3Scale(direction, scale));
    DrawLine3D(origin, end, color);
    // DrawSphere(end, 0.3f * scale, color);  // Arrow head
}

void drawDrone(Eigen::Vector3f position, Eigen::Quaternionf orientation ){
    Vector3 center = toRaylib(position);

    rlPushMatrix();
    rlTranslatef(center.x, center.y, center.z);

    // Kvaternio → rotaatiomatriisi
    Eigen::Matrix3f rotMat = orientation.toRotationMatrix();
    
    // Transponoi matriisi (käänteinen rotaatio)
    rotMat.transposeInPlace();
    
    // Muunna raylib-koordinaatistoon
    float m[16] = {
        rotMat(0,0), rotMat(0,2), rotMat(0,1), 0,
        rotMat(2,0), rotMat(2,2), rotMat(2,1), 0,
        rotMat(1,0), rotMat(1,2), rotMat(1,1), 0,
        0, 0, 0, 1
    };
    rlMultMatrixf(m);

    DrawCube({0, 0, 0}, 0.6f, 0.1f, 0.3f, DARKGRAY);
    DrawCubeWires({0, 0, 0}, 0.6f, 0.1f, 0.3f, BLACK);

    // Propellit - eri väreillä
    const float arm = 0.3f;
    Eigen::Vector3f offsets[4] = {
        {-arm, -arm, 0}, {arm, -arm, 0},
        {-arm, arm, 0}, {arm, arm, 0}
    };
    
    Color propellerColors[4] = {
        RED,    // BL (Back-Left)
        GREEN,  // BR (Back-Right)
        BLUE,   // FL (Front-Left)
        PINK   // FR (Front-Right)
    };

    for (int i = 0; i < 4; i++) {
        Vector3 rp = toRaylib(offsets[i]);
        DrawLine3D({0, 0, 0}, rp, propellerColors[i]);
        DrawSphere(rp, 0.06f, propellerColors[i]);
    }

    rlPopMatrix();

    // Nuolet
    Eigen::Vector3f x_world = orientation * Eigen::Vector3f(1,0,0);
    Eigen::Vector3f y_world = orientation * Eigen::Vector3f(0,1,0);
    Eigen::Vector3f z_world = orientation * Eigen::Vector3f(0,0,1);

    drawArrow(center, toRaylib(x_world.normalized()), RED, 1.0f);
    drawArrow(center, toRaylib(y_world.normalized()), GREEN, 1.0f);
    drawArrow(center, toRaylib(z_world.normalized()), BLUE, 1.0f);
}

void RaylibGraphics::cast()
{
    BeginDrawing();
    ClearBackground(RAYWHITE);
    BeginMode3D(camera);

    // Draw grid and coordinate axes at origin
    DrawGrid(30, 5.0f); // 20x20 ruudukko, ruudun koko 1.0
    DrawLine3D((Vector3){0,0,0}, (Vector3){25,0,0}, RED);   // X
    DrawLine3D((Vector3){0,0,0}, (Vector3){0,25,0}, GREEN); // Y
    DrawLine3D((Vector3){0,0,0}, (Vector3){0,0,25}, BLUE);  // Z

    SimSnapshot simSnapshot = simThread->getSimSnapshot();

   for (auto droneWholeSnapshot : simSnapshot.droneWholeSnapshots){
        
    // ===== hardMissile =====
        Vector3f dronePos = droneWholeSnapshot.position;
        Eigen::Quaternionf orientation = droneWholeSnapshot.orientation;

        drawDrone(dronePos, orientation);
   }

    EndMode3D();

    EndDrawing();
}

void RaylibGraphics::updateCamera()
{
    // Toggle follow mode with T key
    if (IsKeyPressed(KEY_T)) {
        followDrone = !followDrone;
    }

    SimSnapshot simSnapshot = simThread->getSimSnapshot();

    if (followDrone && !simSnapshot.droneWholeSnapshots.empty()) {
        // Kolmannen persoonan kamera
        Vector3f dronePos = simSnapshot.droneWholeSnapshots[0].position;
        Eigen::Quaternionf orientation = simSnapshot.droneWholeSnapshots[0].orientation;
        
        // Kameran etäisyys ja korkeus dronesta
        float distance = 5.0f;
        float height = 2.0f;
        
        // Laske kameran sijainti dronen takana (dronen -X suunta)
        Eigen::Vector3f backDir = orientation * Eigen::Vector3f(-1, 0, 0);
        Eigen::Vector3f upDir = Eigen::Vector3f(0, 0, 1); // Z ylös drone-koordinaatistossa
        
        Eigen::Vector3f cameraPos = dronePos + backDir * distance + upDir * height;
        
        camera.position = toRaylib(cameraPos);
        camera.target = toRaylib(dronePos);
        camera.up = toRaylib(upDir);
    }
    else {
        // Vapaa kamera kun ei seurata
        Vector3 movement = {
            (IsKeyDown(KEY_W) - IsKeyDown(KEY_S)) * 0.25f,
            (IsKeyDown(KEY_D) - IsKeyDown(KEY_A)) * 0.25f,
            (IsKeyDown(KEY_Q) - IsKeyDown(KEY_E)) * 0.25f
        };
        
        Vector3 rotation = {0.0f, 0.0f, 0.0f};
        
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            rotation.x = GetMouseDelta().x * 0.5f;
            rotation.y = GetMouseDelta().y * 0.5f;
        }
        
        float zoom = GetMouseWheelMove() * 2.0f;

        UpdateCameraPro(&camera, movement, rotation, zoom);
    }
}

void RaylibGraphics::initRaylib()
{
    InitWindow(screenWidth, screenHeight, "Missile Simulation");
}

void RaylibGraphics::closeRaylib() {
    CloseWindow();
}