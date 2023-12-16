using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Threading;

/// <summary>
/// Class <c>Device</c> manages variables used for control and observation of the automatically assembled tensegrity manipulator.
/// For use, attach this script to the Empty object having Class <c>Assembly</c>.
/// </summary>
///
public class Device : MonoBehaviour
{
    // **NOTE**
    // This script has to have a lower priority than the Assembly script

    // information to be provided
    public float[] input;
    public float[] cylinder;
    public Vector3[] loopPosition;
    public Vector3[] strutPosition;
    public Quaternion[] strutOrientation;

    // parameters
    public float maxDelta = 1.0f;
    private float rangeSpringCoeff = 800f;
    private float biasSpringCoeff = 240f;


    // tm's parameters
    [HideInInspector]
    public int numLayer;
    [HideInInspector]
    public int numPrism;

    // tm's components
    private SpringJoint[] springs;
    private GameObject[] struts;
    private Assembly asb;

    public int enc(int layer, int prism, int numSpring)
    {
        // check parameters' validity
        if (numSpring == 1) {
            return -1;
        }
        if (prism+numPrism < 0 || prism-numPrism >= numPrism) {
            return -1;            
        }
        if (layer < 0 || layer >= numLayer) {
            return -1;
        }
        // compute the index
        var bias = 0;
        if (numSpring == 0) {
            bias = 2*numPrism;
        }
        return (numPrism*2)*layer + 2*(prism+(numSpring+1)%2) + (layer+numSpring+1)%2 + bias;
    }

    public (int i, int j, int k) dec(int index)
    {
        // check parameters' validity
        if (index >= numLayer*numPrism*2){
            return (-1,-1,-1);
        }
        // compute the i, j, and k
        var jbias = 0;
        var kbias = 0;
        if ( index/8 == 0 ) {
            jbias = (index+1)%2;
            kbias = (index+1)%2*3;
        }
        var i = Math.Max(0, index/8 - (index/8 + index%8 + 1)%2);
        var j = index%8/2 - 1 + jbias;
        var k = (index/8 + index%8)%2*2 + kbias;
        return (i,j,k);
    }
    
    // Start is called before the first frame update
    void Start()
    {
        asb = this.GetComponent<Assembly>();
        numLayer = asb.numLayer;
        numPrism = asb.numPrism;
        struts = new GameObject[numLayer*numPrism];
        cylinder = new float[numLayer*numPrism*2];
        input = new float[numLayer*numPrism*2];
        loopPosition = new Vector3[numLayer+1];
        strutPosition = new Vector3[numLayer*numPrism];
        strutOrientation = new Quaternion[numLayer*numPrism];

        if (asb.isReady) {
            springs = this.GetComponentsInChildren<SpringJoint>();
            for (int i = 0; i < numLayer*numPrism*2; i++) {
                var idx = dec(i);
                cylinder[i] = springs[4*asb.index(idx.i,idx.j)+idx.k].spring;
                input[i] = (cylinder[i] - biasSpringCoeff)/rangeSpringCoeff;
            }
            for (int i = 0; i < numLayer*numPrism; i++) {
                struts[i] = transform.Find($"Strut{i}").gameObject;
            }
        } else {
            throw new Exception("TM is not ready!");
        }
    }

    // FixedUpdate is called once per physical simulation step
    void FixedUpdate()
    {
        // translate the input
        for (int i = 0; i < numLayer*numPrism*2; i++) {
            input[i] = Math.Max(0f,input[i]);
            input[i] = Math.Min(input[i],1f);
            cylinder[i] = rangeSpringCoeff*(input[i]) + biasSpringCoeff;
        }
        // update spring coefficients
        for (int i = 0; i < numLayer*numPrism*2; i++) {
            var idx = dec(i);
            if (Math.Abs(cylinder[i] - springs[4*asb.index(idx.i,idx.j)+idx.k].spring) > maxDelta) {
                springs[4*asb.index(idx.i,idx.j)+idx.k].spring += Math.Sign(cylinder[i] - springs[4*asb.index(idx.i,idx.j)+idx.k].spring) * maxDelta;
            } else {
                springs[4*asb.index(idx.i,idx.j)+idx.k].spring = cylinder[i];
            }
        }
        // retrieve positions of loops' centers
        for (int i = 0; i < numLayer+1; i++) {
            Vector3 ave = new Vector3(0f, 0f, 0f);
            var num = 0;
            for (int bias = 0; bias < 2; bias++) {
                if (i-bias >= 0 && i-bias < numLayer) {
                    for (int j = 0; j < numPrism; j++) {
                        ave += struts[asb.index((i-bias), j)].transform.GetChild(1-bias).transform.position;
                        num++;
                    }
                }
            }
            loopPosition[i] = ave/num;
        }
        //
        for (int i = 0; i < numLayer*numPrism; i++) {
            strutPosition[i] = struts[i].transform.position;
            strutOrientation[i] = struts[i].transform.rotation;
        }
    }
}
