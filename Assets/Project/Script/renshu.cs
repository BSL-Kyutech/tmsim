using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;

public class renshu : MonoBehaviour
{
    public GameObject basePlate;
    public GameObject baseStrut;

    private GameObject[] baseblocks;

    // Start is called before the first frame update
    void Start()
    {
        /*
        GameObject cube=GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.AddComponent<Rigidbody>();
        cube.transform.position=new Vector3(0,0,0);

        Instantiate(basePlate,this.transform);
        
        */


        float radiusBase=1.0f;
        baseblocks = new GameObject[1];

        baseblocks[0] = Instantiate(basePlate, this.transform);                             //basePlateプレハブからobjectを生成し，先ほどの配列に格納
            baseblocks[0].transform.position = this.transform.position + new Vector3(           //baseblocksの座標の配置 
                radiusBase*(float)Math.Cos(4),                                             //x座標はradiusBase(半径)×cos((2π/ストラットの数)×i)極座標→直交座標への変換プロセス
                0,                                                                              //y=0(土台のため)
                radiusBase*(float)Math.Sin(4));
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
