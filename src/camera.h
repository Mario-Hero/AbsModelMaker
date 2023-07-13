#ifndef CAMERA_H
#define CAMERA_H

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <iostream>
#include <vector>

// Defines several possible options for camera movement. Used as abstraction to stay away from window-system specific input methods
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 3.5f;
const float SENSITIVITY = 0.001f;
const float ZOOM = 110.0f;

// An abstract camera class that processes input and calculates the corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
public:
    // camera Attributes
    glm::vec3 Position;
    glm::vec3 addPosition;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    // euler Angles
    float Yaw;
    float Pitch;
    // camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;
    float ballCameraR = 10.f;
    float ballCameraA = 0.5f;
    float ballCameraB = 0.5f;

    // constructor with vectors
    void updatePosition() {
        const float z = cos(ballCameraA) * cos(ballCameraB) * ballCameraR;
        const float x = sin(ballCameraA) * cos(ballCameraB) * ballCameraR;
        const float y = sin(ballCameraB) * ballCameraR;
        Position = glm::vec3(x, y, z) + addPosition;
    }
    Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW, float pitch = PITCH) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        updatePosition();
        updateCameraVectors();
    }
    // constructor with scalar values
    Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch) : Front(glm::vec3(0.0f, 0.0f, -1.0f)), MovementSpeed(SPEED), MouseSensitivity(SENSITIVITY), Zoom(ZOOM) {
        Position = glm::vec3(posX, posY, posZ);
        WorldUp = glm::vec3(upX, upY, upZ);
        Yaw = yaw;
        Pitch = pitch;
        updateCameraVectors();
    }

    // returns the view matrix calculated using Euler Angles and the LookAt Matrix
    glm::mat4 GetViewMatrix() {
        return glm::lookAt(Position, addPosition, Up);
    }

    // processes input received from any keyboard-like input system. Accepts input parameter in the form of camera defined ENUM (to abstract it from windowing systems)
    /*
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        const float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)
            addPosition += Up * velocity;
        if (direction == BACKWARD)
            addPosition -= Up * velocity;
        if (direction == LEFT)
            addPosition -= Right * velocity;
        if (direction == RIGHT)
            addPosition += Right * velocity;
        updatePosition();
        updateCameraVectors();
    }
    */
    void ProcessMouseRightMove(float xoffset, float yoffset) {
        xoffset *= MouseSensitivity * ballCameraR * 0.1f;
        yoffset *= MouseSensitivity * ballCameraR * 0.1f;
        addPosition -= Up * yoffset + Right * xoffset;
        updatePosition();
        updateCameraVectors();
    }
    void ProcessBallMove(float xoffset, float yoffset) {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;
        ballCameraA -= xoffset;
        ballCameraB -= yoffset;
        updatePosition();
        updateCameraVectors();
    }
    // processes input received from a mouse scroll-wheel event. Only requires input on the vertical wheel-axis
    void ProcessMouseScroll(float yoffset) {
        if (yoffset > 0) {
            ballCameraR += 5 * yoffset;
        } else {
            if (ballCameraR < 10) {
                ballCameraR += 0.1f * yoffset;
            } else {
                ballCameraR += 5 * yoffset;
            }
        }
        updatePosition();
    }

private:
    // calculates the front vector from the Camera's (updated) Euler Angles
    void updateCameraVectors() {
        // calculate the new Front vector
        Front = glm::normalize(addPosition - Position);
        // also re-calculate the Right and Up vector
        Right = glm::normalize(glm::cross(Front, WorldUp));  // normalize the vectors, because their length gets closer to 0 the more you look up or down which results in slower movement.
        Up = glm::normalize(glm::cross(Right, Front));
    }
};
#endif
