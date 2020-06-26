using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;



namespace ARRobotInteraction.Base
{
    /// <summary>
    /// A class that dispatches tasks to the main thread.
    /// </summary>
    public class MainThreadDispatcher : MonoBehaviour
    {
        public static MainThreadDispatcher Instance;

        public void Awake()
        {
            Instance = this;
        }

        private List<Action> actionsToExecute = new List<Action>();

        /// <summary>
        /// Add an action to the Que
        /// </summary>
        /// <param name="action"></param>
        public void AddItem(Action action)
        {
            actionsToExecute.Add(action);
        }

        /// <summary>
        /// Process one que-item every frame
        /// </summary>
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