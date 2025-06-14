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

    //ばね定数の変更を自分でやるための変数
    public bool randomInput;
    public bool manualInput;
    //manual Input　どのシリンダーへ入力するか．
    public int cylinderNum;
    public float sprringInput=0.0f;

    public bool standUp;

    //breakingMode(一部破損時状態で行う際のモード)
    public bool breaking_Mode;
    public int breakNum;
    public float BrokenInput=0;
    public bool ConstantBroken=true;


    System.Random r = new System.Random(42);


    //破壊されたシリンダの記録
    private bool[] brokenCylinder;
    //壊れたシリンダの記録をリセット
    public bool resetsBrokenCylinder;


    // Start is called before the first frame update
    void Start()
    {
        //入力
        input = new double[dev.numLayer*dev.numPrism*2];
        for (int i = 0; i < dev.numLayer*dev.numPrism*2; i++) {
            input[i] = 0.5f;
        }
        //角速度と角加速度？
        theta = new double[dev.numLayer];
        omega = new double[dev.numLayer];
        for (int i = 0; i < dev.numLayer; i++) {
            theta[i] = 0.0;
            //omega[i] = 1 + r.NextDouble(); // 1+ random number (0.0<random number <1.0)
            omega[i] = 1.2;
            //Debug.Log(omega[i]);
        }

        //破壊扱いのシリンダの記憶配列
        brokenCylinder=new bool[dev.numLayer*dev.numPrism*2];
        for(int i=0;i<(dev.numLayer*dev.numPrism*2);i++){
            brokenCylinder[i]=false;
        }
    }

    // FixedUpdate is called once per physical simulation step
    void FixedUpdate()
    {

        //壊れた状態のシリンダの記憶
        if(breaking_Mode){
            brokenCylinder[breakNum]=true;
        }
        if(resetsBrokenCylinder){
            for(int i=0;i<dev.numLayer*dev.numPrism*2;i++){
                brokenCylinder[i]=false;
            }
            resetsBrokenCylinder=false;
        }
        //長い配列(dev.inputと同じ長さ)を用意 bool型
        //リセット用モードをつくる

        
        

        
        if (isActive) {
            for (int i = 0; i < dev.numLayer; i++) {
                for (int j = 0; j < 2*dev.numPrism; j++) {
                    
                    input[i*2*dev.numPrism + j] = Math.Cos(theta[i] + (Math.PI/dev.numPrism)*j);
                    //COSでマイナスに行っても大丈夫なように 0.5が最大値になっているわけか.
                    input[i*2*dev.numPrism + j] = (input[i*2*dev.numPrism + j] + 1f)/2f;

                

                    if(breaking_Mode && ConstantBroken &&brokenCylinder[i*2*dev.numPrism + j]==true){
                        dev.input[i*2*dev.numPrism + j] = BrokenInput;
                    }
                    else if(breaking_Mode && brokenCylinder[i*2*dev.numPrism + j]==true){
                        dev.input[i*2*dev.numPrism + j] = (float)(1 + r.NextDouble());
                        //Debug.Log((i*2*dev.numPrism + j).ToString()+"  "+brokenCylinder[i*2*dev.numPrism + j].ToString());
                    }
                    else{
                        dev.input[i*2*dev.numPrism + j] = (float)input[i*2*dev.numPrism + j];
                    }

                    
                }
                theta[i] += omega[i]*Time.fixedDeltaTime;
            }
            
        }//ランダム入力

        if(randomInput){
            //入力
            
            input = new double[dev.numLayer*dev.numPrism*2];
            for (int i = 0; i < 2*dev.numLayer*dev.numPrism; i++) {
                /*
                if(r.Next(2)==1){
                    input[i] = 0.5;
                    dev.input[i] = (float)input[i];
                }
                else{
                    input[i] = 0;
                    dev.input[i] = (float)input[i];
                }
                */
                input[i] = r.NextDouble();

                
                dev.input[i]=(float)input[i];
                
                //Debug.Log(omega[i]);
            }
            randomInput=false;
        }
        /*
        if(randomInput){
            //入力
            input = new double[dev.numLayer*dev.numPrism*2];
            for (int i = 0; i < dev.numLayer*dev.numPrism*2; i++) {
                input[i] = 0.5f;
            }
            for (int i = 0; i < dev.numLayer; i++) {
                omega[i] = 1 + r.NextDouble(); // 1+ random number (0.0<random number <1.0)
                for (int j = 0; j < 2*dev.numPrism; j++) {
                    input[i*2*dev.numPrism + j] = Math.Cos(omega[i]+(Math.PI/dev.numPrism)*j);
                    input[i*2*dev.numPrism + j] = (input[i*2*dev.numPrism + j] + 1f)/2f;
                    dev.input[i*2*dev.numPrism + j] = (float)input[i*2*dev.numPrism + j];
                }

                //Debug.Log(omega[i]);
            }
            randomInput=false;
        }*/
        
        if(manualInput){
            //0を入れるシリンダーの数字の決定とでかい数を入れる数字の決定
            dev.input[cylinderNum]=sprringInput;
            manualInput=false;
        }
        
        if(standUp){
            //入力
            for (int i = 0; i < dev.numLayer; i++) {
                for (int j = 0; j < 2*dev.numPrism; j++) {
                    dev.input[i*2*dev.numPrism + j] =  0.5f;;
                }
            
                //Debug.Log(omega[i]);
            }
            standUp=false;
            
        }

        
        
    }
}
