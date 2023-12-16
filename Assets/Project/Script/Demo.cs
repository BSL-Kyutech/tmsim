using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;

public class Demo : MonoBehaviour
{
    public Device dev;
    private double[] input;
    private double[] orig;
    public bool isActive;

    public double[] omega;
    public double[] theta;

    System.Random r = new System.Random(1000);

    // Start is called before the first frame update
    void Start()
    {
        input = new double[dev.numLayer*dev.numPrism*2];
        for (int i = 0; i < dev.numLayer*dev.numPrism*2; i++) {
            input[i] = 0.5f;
        }
        theta = new double[dev.numLayer];
        omega = new double[dev.numLayer];
        for (int i = 0; i < dev.numLayer; i++) {
            theta[i] = 0.0;
            omega[i] = 1 + r.NextDouble();
        }
    }

    // FixedUpdate is called once per physical simulation step
    void FixedUpdate()
    {
        if (isActive) {
            for (int i = 0; i < dev.numLayer; i++) {
                for (int j = 0; j < 2*dev.numPrism; j++) {
                    input[i*2*dev.numPrism + j] = Math.Cos(theta[i] + (Math.PI/dev.numPrism)*j);
                    input[i*2*dev.numPrism + j] = (input[i*2*dev.numPrism + j] + 1f)/2f;
                    dev.input[i*2*dev.numPrism + j] = (float)input[i*2*dev.numPrism + j];
                }
                theta[i] += omega[i]*Time.fixedDeltaTime;
            }
        }
    }
}
