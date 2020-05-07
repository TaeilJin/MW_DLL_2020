using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

public class MW_Connect : MonoBehaviour
{
    [DllImport("fbxRW")]
    private static extern int INIT_bChaGroup();
    [DllImport("fbxRW")]
    private static extern void SET_JOINT_QUATERNION_MW(string jointname, float[] input_Quat);
    [DllImport("fbxRW")]
    private static extern void SET_JOINT_QUATERNION_UNITY(string jointname, float[] output_Quat);
    [DllImport("fbxRW")]
    private static extern void SET_BASE_POSITION_MW(float[] input_pos);
    [DllImport("fbxRW")]
    private static extern void SET_BASE_POSITION_UNITY(float[] output_POS);

    [DllImport("fbxRW")]
    private static extern void DO_FULLBODY_IK(float[] input_D_RArm, float[] input_D_LArm, float[] input_D_base, float[] input_D_RLeg, float[] input_D_LLeg);

    [DllImport("fbxRW")]
    private static extern IntPtr INIT_JOINT_LIST(int i);

    // Start is called before the first frame update


    public Transform input_angle;

    public Transform DesiredRArm;
    public Transform DesiredLArm;
    public Transform DesiredBase;
    public Transform DesiredRLeg;
    public Transform DesiredLLeg;
    public int num_avatar;
    float[] MakePositionPTR(Vector3 position)
    {
        float[] pos_input = new float[3];
        Vector3 pos_base = position;
        pos_input[0] = pos_base.x; pos_input[1] = pos_base.y; pos_input[2] = pos_base.z;
        return pos_input;
    }
    void UpdateJnt_ByName(string name)
    {
        Quaternion quat = new Quaternion();
        float[] quat_float = new float[4];

        SET_JOINT_QUATERNION_UNITY(name, quat_float);

        quat.Set(quat_float[0], quat_float[1], quat_float[2], quat_float[3]);

        GameObject.Find(name).transform.localRotation = quat;
    }
    void updateJnt_WholeQ(int numlink)
    {
        for (int i = 0; i < numlink; i++)
        {
            string s_joint = Marshal.PtrToStringAnsi(INIT_JOINT_LIST(i));
            UpdateJnt_ByName(s_joint);
        }
    }
    void Start()
    {
        Debug.Log("DLL_Imort:FbxRW ");

        num_avatar = INIT_bChaGroup();

        string s_joint = Marshal.PtrToStringAnsi(INIT_JOINT_LIST(0));
        Debug.Log(s_joint);

        



        Debug.Log("DLL_Import:finshed");
    }

    // Update is called once per frame
    void Update()
    {

        DO_FULLBODY_IK(MakePositionPTR(DesiredRArm.position),
            MakePositionPTR(DesiredLArm.position),
            MakePositionPTR(DesiredBase.position),
            MakePositionPTR(DesiredRLeg.position),
            MakePositionPTR(DesiredLLeg.position));

        updateJnt_WholeQ(52);
        ////position 받아오기
        //float[] pos_input = new float[3];
        //Vector3 pos_base = input_angle.transform.position;
        //pos_input[0] = pos_base.x; pos_input[1] = pos_base.y; pos_input[2] = pos_base.z;
        //SET_BASE_POSITION_MW(pos_input);

        //float[] pos_input2 = new float[3];
        //SET_BASE_POSITION_UNITY(pos_input2);
        //Vector3 pos_output = new Vector3();
        //pos_output.Set(pos_input2[0], pos_input2[1], pos_input2[2]);
        //GameObject.Find("mixamorig:Hips").transform.position = pos_output;



        //Quaternion quat;
        //quat = input_angle.localRotation;
        //float[] quat_float = new float[4];
        //quat_float[0] = quat.x; quat_float[1] = quat.y; quat_float[2] = quat.z; quat_float[3] = quat.w; 
        //SET_JOINT_QUATERNION_MW("mixamorig:LeftArm", quat_float);

        //UpdateJnt_ByName("mixamorig:LeftArm");

    }
}
