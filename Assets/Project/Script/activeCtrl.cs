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
    public bool AllRandom=false;
    public bool Allbias=false;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

        float timeNow=Time.realtimeSinceStartup;
        if(timeNow>=5.0f){
            Allactive=true;
        }
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
        
        if(AllRandom==true){
            ctrl1.randomInput=true;
            ctrl2.randomInput=true;
            ctrl3.randomInput=true;
            AllRandom=false;
        }
        else if(AllRandom==false){
            ctrl1.randomInput=false;
            ctrl2.randomInput=false;
            ctrl3.randomInput=false;
        }
        
        if(Allbias==true){
            ctrl1.manualInput=true;
            ctrl2.manualInput=true;
            ctrl3.manualInput=true;
            Allbias=false;
        }
        else if(Allbias==false){
            ctrl1.manualInput=false;
            ctrl2.manualInput=false;
            ctrl3.manualInput=false;
        }
        
        
    }
}
