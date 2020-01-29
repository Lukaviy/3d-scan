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

struct CamBufferData
{
    public double rx;
    public double ry;
    public double rz;

    public double tx;
    public double ty;
    public double tz;
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

    public static Quaternion VectorToQuaternion(Vector3 m)
    {
        float theta = (float)(Math.Sqrt(m.x*m.x + m.y*m.y + m.z*m.z)*180/Math.PI);
        Vector3 axis = new Vector3 (m.x, m.y, m.z);
        return Quaternion.AngleAxis (theta, axis);
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
        var pos = new Vector3(-(float) d.tx, (float) d.ty, (float) d.tz);
        var rot = VectorToQuaternion(new Vector3((float)d.rx, (float)d.ry, (float)d.rz));
        return new CamData { position = pos, rotation = rot };
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
                var d = fromBytes(receiveBytes);

                _camData[0] = parseCamBufferData(d.cam1);
                _camData[1] = parseCamBufferData(d.cam2);

                Debug.Log($"{ _camData[0].position.x }, { _camData[0].position.y }, { _camData[0].position.z }");

                dataProceeded = true;
            }
        }
    }
}
