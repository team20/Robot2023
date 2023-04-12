// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.ArrayList;

/** Add your docs here. */
public class PixyCamObjectMap {

    private PixyCamObject[] m_objectMap = new PixyCamObject[256];

    public synchronized PixyCamObject get(int index){
        return m_objectMap[index];
        // try{
        //     PixyCamObject obj = m_objectMap.get(index);
        //     if(obj.isExpired()){
        //         obj = null;
        //         m_objectMap.set(index, null);
        //     }
        //     return obj;
        // }catch(Exception e){
        //     return null;
        // }
    }

    public synchronized void set(int index, PixyCamObject element){
        m_objectMap[index]= element;
    }
    // public synchronized void add(int index, PixyCamObject element){
    //     m_objectMap.add(index, element);
    // }

    public synchronized int size(){
        return m_objectMap.length;
    }
}
