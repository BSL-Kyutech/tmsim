using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class activeCtrl : MonoBehaviour
{
    //ctrlのオブジェクトたち
    public Demo ctrl1;
    public Demo ctrl2;
    public Demo ctrl3;

    //すべてをONにするかどうかの変数
    public bool Allactive=false;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if(Allactive==true){
            ctrl1.isActive=true;
            ctrl2.isActive=true;
            ctrl3.isActive=true;
        }
        else if(Allactive==false){
            ctrl1.isActive=false;
            ctrl2.isActive=false;
            ctrl3.isActive=false;
        }
    }
}
