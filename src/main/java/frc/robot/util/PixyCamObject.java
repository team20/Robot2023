// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.time.Duration;
import java.time.Instant;

/** Add your docs here. */
public class PixyCamObject{
    private final int kTimeValid = 50;
    private int m_signature;
    private int m_x;
    private int m_y;
    private int m_width;
    private int m_height;
    private int m_index;
    private int m_age;
    private Instant m_readTime;

    public PixyCamObject(int signature, int x, int y, int width, int height, int index, int age){
        m_signature = signature;
        m_x = x;
        m_y = y;
        m_width = width;
        m_height = height;
        m_index = index;
        m_age = age;
        m_readTime = Instant.now();
        
    }

    public synchronized boolean isExpired(){
        return Duration.between(m_readTime, Instant.now()).toMillis() > kTimeValid;
    }

    public synchronized int[] get(){
        int[] vals = {m_signature, m_x, m_y, m_width, m_height, m_index, m_age};
        return vals;
    }

    public synchronized void set(int m_signature, int m_x, int m_y, int m_width, int m_height, int m_index, int m_age) {
        this.m_signature = m_signature;
        this.m_x = m_x;
        this.m_y = m_y;
        this.m_width = m_width;
        this.m_height = m_height;
        this.m_index = m_index;
        this.m_age = m_age;
        this.m_readTime = Instant.now();
    }
}