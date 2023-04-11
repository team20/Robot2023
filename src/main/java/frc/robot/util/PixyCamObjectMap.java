// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

/** Add your docs here. */
public class PixyCamObjectMap {



    private ArrayList<PixyCamObject> m_objectMap = new ArrayList<PixyCamObject>();

    public synchronized PixyCamObject get(int index){
        try{
            PixyCamObject obj = m_objectMap.get(index);
            if(obj.isExpired()){
                obj = null;
                m_objectMap.set(index, null);
            }
            return obj;
        }catch(Exception e){
            return null;
        }
    }

    public synchronized void set(int index, PixyCamObject element){
        m_objectMap.set(index, element);
    }

    public synchronized int size(){
        return m_objectMap.size();
    }
}
