using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;

/// <summary>
/// Class <c>Assembly</c> automatically instantiates a tensegrity manipulator with designated parameters.
/// For use, place an Empty object to where the base of the tensegrity manipulator should be, and attach this script.
/// </summary>
///
public class Assembly : MonoBehaviour
{
    // Prefabs
    //================================================
    // Expected parameters
    // basePlate: base
    // baseStrut: base_strut
    // middleStrut: strut
    // loopLine: loop_renderer
    // verticalLine: stripe_renderer
    //================================================
    public GameObject basePlate;
    public GameObject baseStrut;
    public GameObject middleStrut;
    public GameObject loopLine;
    public GameObject vertcalLine;

    // Public Member Variables
    //================================================
    // Recommended parameters for TM40
    // numLayer = 5
    // numPrism = 4
    // radiusBase = 0.22
    // edgeLoop = {0.05, 0.04, 0.03, 0.02, 0.05}
    // *) The length of loopEdge has to be equal to numLayer, 
    //    otherwise it will be ignored.
    //================================================
    public int numLayer;
    public int numPrism;
    public float radiusBase;
    public float[] edgeLoop;
    public bool isPausedStart;
    public bool isNeverSleep;

    // Array to keep struts
    //================================================
    // Because this array holds almost all important info,
    // it sets this to public but keeps hidden for the inspector.
    // For instance, you acess this array from other script 
    // like this:
    //-----------------------------------------
    // //<<As class member:>>
    // public GameObject tmObject;
    // //<<After assigning the tm40 prefab's instance to tmObject>>
    // Assembly tm = tm.GetComponent<Assembly>;
    // // you can access to the i-th strut as tm.struts[i]
    //================================================
    [HideInInspector]
    public GameObject[] struts;

    // Array to keep other instances of prefabs.
    private GameObject[] baseblocks;
    private GameObject[] loops;
    private GameObject[] stripes;
    
    // Flag that all instances are generated
    [HideInInspector]
    public bool isReady = false;

    // Generate the strut number at the j-th strut in the i-th layer
    public int index(int i, int j)
    {
        if (j < 0) {
            j = j + numPrism;
        } else if (j >= numPrism) {
            j = j - numPrism;
        }
        if (j < 0 || j >= numPrism){
            return -1;
        } else {
            return numPrism * i + j;
        }
    }
    
    // Start is called before the first frame update
    void Start()
    {
        // Allocate variables
        baseblocks = new GameObject[numPrism];
        struts = new GameObject[numPrism*numLayer];
        loops = new GameObject[(numLayer+1)];
        stripes = new GameObject[numPrism*2];
        
        // Place the bottom layer's parts and connect ball joints
        for (int i = 0; i < numPrism ; i++)
        {
            var step = 2*Math.PI/numPrism;
            // Instantiate and place prefabs
            baseblocks[i] = Instantiate(basePlate, this.transform);
            baseblocks[i].transform.position = this.transform.position + new Vector3(
                radiusBase*(float)Math.Cos(step*i), 
                0, 
                radiusBase*(float)Math.Sin(step*i));
            baseblocks[i].name = $"Base{i}";
            struts[i] = Instantiate(baseStrut, this.transform);
            struts[i].transform.position = this.transform.position + new Vector3(
                radiusBase*(float)Math.Cos(step*i), 
                0.265f, 
                radiusBase*(float)Math.Sin(step*i)
                );
            struts[i].name = $"Strut{i}";
            // Connect joints
            ConfigurableJoint basejoint = struts[i].GetComponent<ConfigurableJoint>();
            basejoint.connectedBody = baseblocks[i].GetComponent<Rigidbody>();
        }

        // Place the rest layers' struts
        for (int i = 1; i < numLayer ; i++)
        {
            var step = 2*Math.PI/numPrism;
            var twist = Math.PI/numPrism;
            // Instantiate and place the prefab
            for (int j = 0; j < numPrism ; j++)
            {
                struts[index(i, j)] = Instantiate(middleStrut, this.transform);
                struts[index(i, j)].transform.position = this.transform.position + new Vector3(
                    radiusBase/1.5f*(float)Math.Cos(step*j+twist*(i%2)),
                    0.265f+(float)(0.3*i), 
                    radiusBase/1.5f*(float)Math.Sin(step*j+twist*(i%2))
                    );
                struts[index(i, j)].name = $"Strut{index(i,j)}";
            }
        }
        
        var isMatched = edgeLoop.GetLength(0) == numLayer;
        // Connect & configure spring joints
        for (int i = 0; i < numLayer ; i++) 
        {
            // prepare the variable to express the twisting direction in the i-th layer
            var twist_dir = -1;
            if (i%2 == 0) {
                twist_dir = 1;
            }

            // connect springs
            for (int j = 0; j < numPrism ; j++)
            {
                SpringJoint[] spring = struts[index(i, j)].GetComponents<SpringJoint>();
                
                // Each strut has 4 springs
                // prepare target indice
                // *** Layer-dependent exceptions will be addressed later ***
                
                var target1 = index(i+2, j+twist_dir);
                var target2 = index(i+1, j);
                var target3 = index(i, j+twist_dir);
                var target4 = index(i-1, j-twist_dir);

                // Connect springs while dealing with layer-dependent exceptions
                // #1
                if (i == numLayer-1) {
                    target1 = index(i, j+twist_dir);
                    spring[0].spring = 0.0f;
                    spring[0].connectedAnchor = new Vector3(0, 1, 0);
                } else if (i == numLayer-2) {
                    target1 = index(i+1, j+twist_dir);
                    spring[0].connectedAnchor = new Vector3(0, 1, 0);
                } else {
                    spring[0].connectedAnchor = new Vector3(0, -1, 0);
                }
                spring[0].anchor = new Vector3(0, 1, 0);
                spring[0].connectedBody = struts[target1].GetComponent<Rigidbody>();
                
                // #2
                if (i == numLayer-1) {
                    target2 = index(i, j+twist_dir);
                    spring[1].connectedAnchor = new Vector3(0, 1, 0);
                } else {
                    spring[1].connectedAnchor = new Vector3(0, -1, 0);
                }
                spring[1].anchor = new Vector3(0, 1, 0);
                spring[1].connectedBody = struts[target2].GetComponent<Rigidbody>();
                if (isMatched) {
                    spring[1].maxDistance = edgeLoop[i];
                }
                
                // #3
                spring[2].connectedBody = struts[target3].GetComponent<Rigidbody>();
                spring[2].connectedAnchor = new Vector3(0, -1, 0);
                spring[2].anchor = new Vector3(0, 1, 0);

                // #4
                if (i == 0 && i != numLayer-1) {
                    target4 = index(i+1, j-2*twist_dir);
                    spring[3].connectedAnchor = new Vector3(0, -1, 0);
                } else if (i == 0 && i == numLayer-1) {
                    target4 = index(i, j-twist_dir);
                    spring[3].connectedAnchor = new Vector3(0, -1, 0);
                    spring[3].spring = 0.0f;
                } else {
                    spring[3].connectedAnchor = new Vector3(0, 1, 0);
                    if (isMatched) {
                        spring[3].maxDistance = edgeLoop[i-1];
                    }
                }
                spring[3].connectedBody = struts[target4].GetComponent<Rigidbody>();
                spring[3].anchor = new Vector3(0, -1, 0);
            }
        }

        // Add loop renderers
        for (int i = 0; i <= numLayer ; i++) 
        {
            loops[i] = Instantiate(loopLine, this.transform);
            loops[i].name = $"Loop Renderer{i}";
            LineRenderer line = loops[i].GetComponent<LineRenderer>();
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.widthMultiplier = 0.005f;
            line.loop = true;
            line.numCapVertices = 1;
            line.numCornerVertices = 1;
            if (i == 0 || i == numLayer) 
            {
                line.positionCount = numPrism;
            } 
            else {
                line.positionCount = numPrism*2;
            }
        }

        // Add stripe renderers
        for (int i = 0; i < numPrism*2 ; i++) 
        {
            stripes[i] = Instantiate(vertcalLine, this.transform);
            stripes[i].name = $"Stripes Renderer{i}";
            LineRenderer line = stripes[i].GetComponent<LineRenderer>();
            line.material = new Material(Shader.Find("Sprites/Default"));
            line.widthMultiplier = 0.005f;
            line.loop = false;
            line.numCapVertices = 1;
            line.numCornerVertices = 1;
            line.positionCount = numLayer+1;
        }
        
        if (isPausedStart) {
            UnityEditor.EditorApplication.isPaused = true;
        }

        if (isNeverSleep) {
            for (int i = 0; i < numLayer; i++) {
                for (int j = 0; j < numPrism; j++) {
                    Rigidbody rb = struts[index(i, j)].GetComponent<Rigidbody>();
                    rb.sleepThreshold = -1;
                }
            }
        }

        isReady = true;
    }

    // Update is called once per frame
    void Update()
    {      
        // Update loop renderers
        for (int i = 0; i <= numLayer ; i++) 
        {
            LineRenderer line = loops[i].GetComponent<LineRenderer>();
            if (i == 0) {
                for (int j = 0; j < numPrism ; j++) {
                    line.SetPosition(j, struts[j].transform.GetChild(1).transform.position);
                }
            } else if (i == numLayer) {
                for (int j = 0; j < numPrism ; j++) {
                    line.SetPosition(j, struts[index(numLayer-1,j)].transform.GetChild(0).transform.position);
                }
            } else {
                if(i%2 == 1){
                    for (int j = 0; j < numPrism ; j++) {
                        line.SetPosition(2*j, struts[index(i-1,j)].transform.GetChild(0).transform.position);
                        line.SetPosition(2*j+1, struts[index(i,j)].transform.GetChild(1).transform.position);
                    }
                } else {
                    for (int j = 0; j < numPrism ; j++) {
                        line.SetPosition(2*j, struts[index(i,j)].transform.GetChild(1).transform.position);
                        line.SetPosition(2*j+1, struts[index(i-1,j)].transform.GetChild(0).transform.position);
                    }
                }
            }
        }

        // Update stripe renderers
        for (int i = 0; i < numPrism ; i++) 
        {
            LineRenderer line1 = stripes[2*i].GetComponent<LineRenderer>();
            LineRenderer line2 = stripes[2*i+1].GetComponent<LineRenderer>();
            line1.SetPosition(0, struts[index(0,i)].transform.GetChild(1).transform.position);
            line2.SetPosition(0, struts[index(0,i+1)].transform.GetChild(1).transform.position);
            for (int j = 1; j <= numLayer ; j++) 
            {
                if ((j/2)*2 < numLayer) {
                    line1.SetPosition(j, struts[index((j/2)*2, i-j%2)].transform.GetChild((j+1)%2).transform.position);
                } else {
                    line1.SetPosition(j, struts[index(numLayer-1, i+1)].transform.GetChild(0).transform.position);
                }
                if (((j-1)/2)*2+1 < numLayer) {
                    line2.SetPosition(j, struts[index(((j-1)/2)*2+1, i-j%2)].transform.GetChild(j%2).transform.position);
                } else {
                    line2.SetPosition(j, struts[index(numLayer-1, i-1)].transform.GetChild(0).transform.position);
                }
            }
        }

    }

}
