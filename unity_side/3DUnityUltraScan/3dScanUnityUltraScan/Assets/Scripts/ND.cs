using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Networking;

public class ND : NetworkDiscovery
{
    public override void OnReceivedBroadcast(string fromAddress, string data)
    {
        data.Where((x) => x != 0);
        Debug.Log("Received broadcast from: " + fromAddress + " with the data: " + data);
        OnReceived(new string(data.Where((x) => x != 0).ToArray()));
    }

    public Action<string> OnReceived { get; set; }
}