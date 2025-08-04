using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.IO;
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

    //broken mode
    //破壊されたシリンダの記録
    public bool[] brokenCylinder;
    //壊れたシリンダの記録をリセット
    public bool resetsBrokenCylinder;

    //breakingMode(一部破損時状態で行う際のモード)
    public bool breaking_Mode;
    public int breakNum;
    public float BrokenInput=0;
    public bool ConstantBroken=true;

    System.Random r = new System.Random(42);

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
        if ( index/(numPrism*2) == 0 ) {
            jbias = (index+1)%2;
            kbias = (index+1)%2*3;
        }
        //奇数と偶数で層の上下を判別
        var i = Math.Max(0, index/(numPrism*2) - (index/(numPrism*2) + index%(numPrism*2) + 1)%2);
        //層による上下の進み方(0,1,2..)のような
        var j = index%(numPrism*2)/2 - 1 + jbias;
        var k = (index/(numPrism*2) + index%(numPrism*2))%2*2 + kbias;
        return (i,j,k);
    }

    //ストラットの二点の取得
    private (Vector3 UpperEndPosition,Vector3 BotomEndPosition) getEndPosition(int num){

        //柱オブジェクトの取得
        var strut=this.gameObject.transform.Find("Strut"+(num).ToString()).gameObject;
        if(strut==null){
            //エラー表示
            throw new ArgumentNullException($"{nameof(strut)}is null.  You must cast existing strut's number in this tm_g!");
        }
        //Debug.Log(num);
        
        //上端の取得
        var UpperEnd=strut.transform.Find("end1");
        //baseStrut(最下層のストラット)のみendという名称のため最下層なら取り直す
        if(UpperEnd==null){
            UpperEnd=strut.transform.Find("end");
        }
        //Debug.Log(UpperEnd.transform.position);
        
        //下端の取得
        var BotomEnd=strut.transform.Find("end2");
        if(BotomEnd==null){
            BotomEnd=strut.transform.Find("ball_joint");
        }
        //Debug.Log(BotomEnd.transform.position);
        
        //エンドの座標取得
        Vector3 UpperEndPosition=UpperEnd.transform.position;
        Vector3 BotomEndPosition=BotomEnd.transform.position;

        return (UpperEndPosition,BotomEndPosition);
    }

    //
    private float mimeticsIMU(){
        //座標の取得
        var ends=getEndPosition(1);
        Vector3 UpperEndPosition=ends.UpperEndPosition;
        Vector3 BotomEndPosition=ends.BotomEndPosition;

        //ストラットの中点
        Vector3 AverageEndPosition=Vector3.Lerp(UpperEndPosition,BotomEndPosition,0.5f);
        //Debug.Log(AverageEndPosition);
        
        //ストラット中点と下端の距離
        float deltaX=AverageEndPosition.x-BotomEndPosition.x;
        float deltaY=AverageEndPosition.y-BotomEndPosition.y;
        float deltaZ=AverageEndPosition.z-BotomEndPosition.z;
        //Debug.Log(deltaX);]
        //Debug.Log(this.gameObject);
        //Debug.Log(deltaY);
        //Debug.Log(MathF.Sqrt(Mathf.Pow(deltaX,2.0f)+Mathf.Pow(deltaZ,2.0f)));

        //角度の計算(IMUの座標は重力方向とストラットの角度のみ計測)
        float angle = 0.0f;
        //x軸角度  tan-1(y/x)
        angle=Mathf.Atan2(MathF.Sqrt(Mathf.Pow(deltaX,2.0f)+Mathf.Pow(deltaZ,2.0f)),deltaY);

        
        ///Debug.Log((float)(180/Math.PI)*angle);


        return angle;
    }

    private void OutputCsv(string path,string savedata ){

        //File.AppendAllText("C:/Users/Yamauchi Gaito/Desktop/workspace/tmsim/data/data.csv",savedata);
        File.AppendAllText(path,savedata);
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

        //string outputNums="i,idx.i,idx.j,idx.k,springNumber,strut\n";

        if (asb.isReady) {
            springs = this.GetComponentsInChildren<SpringJoint>();
            //Debug.Log(springs.Length);
            for (int i = 0; i < numLayer*numPrism*2; i++) {
                var idx = dec(i);
                //Debug.Log(idx);
                //Debug.Log(numPrism*asb.index(idx.i,idx.j)+idx.k);
                cylinder[i] = springs[4*asb.index(idx.i,idx.j)+idx.k].spring;
                //Debug.Log(springs[4*asb.index(idx.i,idx.j)+idx.k].connectedBody);
                input[i] = (cylinder[i] - biasSpringCoeff)/rangeSpringCoeff;
                //outputNums=outputNums+i.ToString()+","+idx.i.ToString()+","+idx.j.ToString()+","+idx.k.ToString()+","+(4*asb.index(idx.i,idx.j)+idx.k).ToString()+","+springs[4*asb.index(idx.i,idx.j)+idx.k].connectedBody.ToString()+"\n";

                
            }
            for (int i = 0; i < numLayer*numPrism; i++) {
                struts[i] = transform.Find($"Strut{i}").gameObject;
            }
        } else {
            throw new Exception("TM is not ready!");
        }

        //破壊扱いのシリンダの記憶配列
        brokenCylinder=new bool[numLayer*numPrism*2];
        for(int i=0;i<(numLayer*numPrism*2);i++){
            brokenCylinder[i]=false;
        }

        //string path= "C:/Users/Yamauchi Gaito/Desktop/workspace/_tmsim/data/strut_idx_iデータ"+numLayer.ToString()+numPrism.ToString()+".csv";
        //OutputCsv(path,outputNums);
    }

    // FixedUpdate is called once per physical simulation step
    void FixedUpdate()
    {

        //壊れた状態のシリンダの記憶
        if(breaking_Mode){
            //brokenCylinder[breakNum]=true;
        }
        if(resetsBrokenCylinder){
            for(int i=0;i<numLayer*numPrism*2;i++){
                brokenCylinder[i]=false;
            }
            resetsBrokenCylinder=false;
        }
        //長い配列(inputと同じ長さ)を用意 bool型
        //リセット用モードをつくる

        mimeticsIMU();
        rangeSpringCoeff=200.0f*(float)numPrism;
        biasSpringCoeff=60.0f*(float)numPrism;
        // translate the input
        for (int i = 0; i < numLayer*numPrism*2; i++) {

            if(breaking_Mode && ConstantBroken &&brokenCylinder[i]==true){
                input[i] = BrokenInput;
            }
            else if(breaking_Mode && brokenCylinder[i]==true){
                input[i] = (float)(r.NextDouble());
                //Debug.Log((i).ToString()+"  "+brokenCylinder[i].ToString());
            }

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
