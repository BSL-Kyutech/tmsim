using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class CameraController : MonoBehaviour
{
    [SerializeField, Range(0.1f, 30.0f)]
    private float rotationSpeed = 10.0f;

    [SerializeField, Range(0.1f, 10.0f)]
    private float movementSpeed = 5.0f;

    private bool enableCameraControl = true;

    // Update is called once per frame
    void Update()
    {
        if (enableCameraControl)
        {
            // Camera Rotation
            float rotation = rotationSpeed * Time.deltaTime;

            if (Input.GetKey(KeyCode.Z))
            {
                transform.Rotate(Vector3.right * rotation);
            }
            else if (Input.GetKey(KeyCode.C))
            {
                transform.Rotate(Vector3.left * rotation);
            }

            if (Input.GetKey(KeyCode.Alpha3)) // counterclockwise
            {
                transform.rotation *= Quaternion.Euler(Vector3.up * rotation);
            }
            else if (Input.GetKey(KeyCode.Alpha1)) // clockwise
            {
                transform.rotation *= Quaternion.Euler(Vector3.down * rotation);
            }

            // Camera Movement
            float horizontalMovement = Input.GetAxis("Horizontal") * movementSpeed * Time.deltaTime;
            float verticalMovement = Input.GetAxis("Vertical") * movementSpeed * Time.deltaTime;

            transform.Translate(new Vector3(horizontalMovement, 0, verticalMovement));

            // Additional Controls
            if (Input.GetKey(KeyCode.W)) // Forward
            {
                transform.Translate(Vector3.forward * movementSpeed * Time.deltaTime);
            }
            else if (Input.GetKey(KeyCode.S)) // Backward
            {
                transform.Translate(Vector3.back * movementSpeed * Time.deltaTime);
            }

            if (Input.GetKey(KeyCode.A)) // Left
            {
                transform.Translate(Vector3.left * movementSpeed * Time.deltaTime);
            }
            else if (Input.GetKey(KeyCode.D)) // Right
            {
                transform.Translate(Vector3.right * movementSpeed * Time.deltaTime);
            }

            if (Input.GetKey(KeyCode.Q)) // Up
            {
                transform.Translate(Vector3.up * movementSpeed * Time.deltaTime);
            }
            else if (Input.GetKey(KeyCode.E)) // Down
            {
                transform.Translate(Vector3.down * movementSpeed * Time.deltaTime);
            }
        }

        // Toggle Camera Control
        if (Input.GetKeyDown(KeyCode.Space))
        {
            enableCameraControl = !enableCameraControl;
        }

        // Reset Camera Rotation
        if (Input.GetKeyDown(KeyCode.P))
        {
            transform.rotation = Quaternion.identity;
        }
    }
}