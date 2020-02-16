using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEngine.UI;

public struct Vector3D
{
    public double v1;
    public double v2;
    public double v3;
}

public struct Vector2D
{
    public double x;
    public double y;
}

public struct Matrix3x3D
{
    public Vector3D r1;
    public Vector3D r2;
    public Vector3D r3;
}

struct CamBufferData
{
    public Matrix3x3D rmat;
    public Vector3D tvec;
    public Vector2D lightDotPos;
}

struct Data
{
    public CamBufferData cam1;
    public CamBufferData cam2;
}

public struct CamData
{
    public Vector3 position;
    public Quaternion rotation;
    public Vector2 lightDotDiretion;
}

public class UDP : MonoBehaviour
{
    static UdpClient udp;
    Thread thread;

    static readonly object lockObject = new object();

    public CamData[] CamData;
    private CamData[] _camData;

    public Text IPAddressText;
    public Text ReadedMessagesText;

    public int GetIndexOfLastMessage()
    {
        return readedMessages;
    }

    private int readedMessages = 0;

    private bool dataProceeded = false;

    private ND _nd;

    void Start()
    {
        thread = new Thread(new ThreadStart(ThreadMethod));
        thread.Start();

        IPAddressText.text = string.Join("\n", IPManager.GetIP(ADDRESSFAM.IPv4).Select(x => x.ToString()));

        CamData = new CamData[2];
        _camData = new CamData[2];

        /* _nd = gameObject.AddComponent<ND>();
 
         _nd.broadcastData = "";
 
         _nd.Initialize();
 
         _nd.StartAsClient();
 
         _nd.OnReceived = (x) => { worldMatrix = fromBytes(System.Convert.FromBase64String(x)); };*/
    }

    Data fromBytes(byte[] arr)
    {
        var data = new Data();

        int size = Marshal.SizeOf(data);
        IntPtr ptr = Marshal.AllocHGlobal(size);
        
        //Debug.Log($"{arr.Length}, {size}");

        Marshal.Copy(arr, 0, ptr, size);

        data = (Data)Marshal.PtrToStructure(ptr, data.GetType());
        Marshal.FreeHGlobal(ptr);

        return data;
    }

    public static Quaternion MatrixToRotation(Matrix4x4 m)
    {
        // Adapted from: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
        Quaternion q = new Quaternion();
        q.w = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2;
        q.x = Mathf.Sqrt(Mathf.Max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2;
        q.y = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2;
        q.z = Mathf.Sqrt(Mathf.Max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2;
        q.x *= Mathf.Sign(q.x * (m[2, 1] - m[1, 2]));
        q.y *= Mathf.Sign(q.y * (m[0, 2] - m[2, 0]));
        q.z *= Mathf.Sign(q.z * (m[1, 0] - m[0, 1]));
        return q;
    }

    public static Matrix4x4 BufferMatrixToMatrix4X4(Matrix3x3D m)
    {
        return new Matrix4x4(
            new Vector4((float)m.r1.v1, (float)m.r1.v2, (float)m.r1.v3, 0), 
            new Vector4((float)m.r2.v1, (float)m.r2.v2, (float)m.r2.v3, 0), 
            new Vector4((float)m.r3.v1, (float)m.r3.v2, (float)m.r3.v3, 0),
            new Vector4(0, 0, 0, 1)
        );
    }

    public static Quaternion VectorToQuaternion2(Vector3 m)
    {
        var theta = m.magnitude;

        var itheta = theta == 0.0f ? 1f/theta : 0f;

        m *= itheta;

        var rrt = new Matrix4x4
        (
            new Vector4(0, -m.z, m.y, 0),
            new Vector4(m.z, 0, -m.x, 0),
            new Vector4(-m.y, m.x, 0, 0),
            new Vector4(0,0,0,1)
        );

        rrt = Matrix4x4.Transpose(rrt);

        var r_x = new Matrix4x4(
            new Vector4(m.x*m.x, m.x*m.y, m.x*m.z, 0),
            new Vector4(m.x*m.y, m.y*m.y, m.y*m.z, 0),
            new Vector4(m.x*m.z, m.y*m.z, m.z*m.z, 0),
            new Vector4(0,0,0,1)
        );

        r_x = Matrix4x4.Transpose(r_x);

        var c = Mathf.Cos(theta);
        var s = Mathf.Sin(theta);
        var c1 = 1 - c;

        //var R = ;
        
        return new Quaternion();
    }

    public static Quaternion MatrixToRotation(Matrix3x3D m)
    {
        //var mat1 = new Matrix4x4
        //(
        //    new Vector4(0, -m.z, m.y, 0),
        //    new Vector4(m.z, 0, -m.x, 0),
        //    new Vector4(-m.y, m.x, 0, 0),
        //    new Vector4(0,0,0,1)
        //);

        //mat1 = Matrix4x4.Transpose(mat1);

        //var mat2 = new Matrix4x4(
        //    new Vector4(m.x*m.x, m.x*m.y, m.x*m.z, 0),
        //    new Vector4(m.x*m.y, m.y*m.y, m.y*m.z, 0),
        //    new Vector4(m.x*m.z, m.y*m.z, m.z*m.z, 0),
        //    new Vector4(0,0,0,1)
        //);

        //mat2 = Matrix4x4.Transpose(mat2);

        //var theta = m.magnitude;

        //var c = Mathf.Cos(theta);
        //var s = Mathf.Sin(theta);

        var mat = BufferMatrixToMatrix4X4(m);

        var basisTransform = new Matrix4x4(
            new Vector4(-1, 0, 0, 0), 
            new Vector4(0, -1, 0, 0), 
            new Vector4(0, 0, 1, 0), 
            new Vector4(0, 0, 0, 1)
        );

        var res = basisTransform.inverse * mat * basisTransform;

        return MatrixToRotation(res);
    }

    void Update()
    {
        if (lockObject != null && dataProceeded)
        {
            for (var i = 0; i < _camData.Length; i++)
            {
                CamData[i] = _camData[i];
            }
            //transform.SetParent(Camera.main.transform);
            //transform.localPosition = position;
            //transform.localRotation = rotation;
            //transform.SetParent(null);
            //transform.SetPositionAndRotation(position, rotation);

            readedMessages++;

            ReadedMessagesText.text = readedMessages.ToString();

            dataProceeded = false;

            //Debug.Log("Received");
        }
    }

    private CamData parseCamBufferData(CamBufferData d)
    {
        var pos = new Vector3((float) -d.tvec.v1, (float) -d.tvec.v2, (float) d.tvec.v3);
        //var rot = Quaternion.Inverse(VectorToQuaternion(new Vector3((float)d.rx, (float)d.ry, (float)d.rz)));

        var rot = MatrixToRotation(d.rmat);

        return new CamData { position = rot * pos, rotation = rot, lightDotDiretion = new Vector2((float)d.lightDotPos.x, (float)d.lightDotPos.y) };
    }

    private void ThreadMethod()
    {
        var broadcastAddress = new IPEndPoint(IPAddress.Any, 7325);
        udp = new UdpClient();
        udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        udp.ExclusiveAddressUse = false;

        udp.Client.Bind(broadcastAddress);
        while (true)
        {
            byte[] receiveBytes = udp.Receive(ref broadcastAddress);

            /*lock object to make sure there data is 
            *not being accessed from multiple threads at the same time*/
            lock (lockObject)
            {
                //Debug.Log($"{ _camData[0].position.x }, { _camData[0].position.y }, { _camData[0].position.z }");

                var d = fromBytes(receiveBytes);

                _camData[0] = parseCamBufferData(d.cam1);
                _camData[1] = parseCamBufferData(d.cam2);

                dataProceeded = true;
            }
        }
    }
}
