using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;



namespace ARRobotInteraction.Base
{
    public class MainThreadDispatcher : MonoBehaviour
    {
        public static MainThreadDispatcher Instance;

        public void Awake()
        {
            Instance = this;
        }

        private List<Action> actionsToExecute = new List<Action>();

        public void AddItem(Action action)
        {
            actionsToExecute.Add(action);
        }

        public void Update()
        {
            if (actionsToExecute.Count > 0)
            {
                actionsToExecute[0]();
                actionsToExecute.RemoveAt(0);
            }
        }

    }
}