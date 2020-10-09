std::vector<dvec2> Topology::extractCriticalPoints(const VectorField2& vectorField, std::vector<dvec2>& cvPos)
{
    std::vector<dvec2> cp;
    
    // if cell is small enough
    if(cvPos[1].x-cvPos[0].x<=cellSize.x && cvPos[1].y-cvPos[0].y<=cellSize.y){
        // change-of-sign check
        std::vector<dvec2> cvVal((int)cvPos.size(), dvec2(0, 0));
        for(int i=0; i<(int)cvVal.size(); i++){
            cvVal[i] = vectorField.interpolate(cvPos[i]);
        }
        
        // if corner point is cp
        for(int i=0; i<(int)cvVal.size(); i++){
            dvec2 velocity = cvVal[i];
            dmat2 jacobian = vectorField.derive(velocity);
            if(-MINVAL<velocity.x && velocity.x<MINVAL && -MINVAL<velocity.y && velocity.y<MINVAL && glm::determinant(jacobian) != 0){
                cp.push_back(cvPos[i]);
            }
        }
        
        if((cvVal[0].x*cvVal[1].x<0 && cvVal[0].y*cvVal[1].y<0) || (cvVal[1].x*cvVal[2].x<0 && cvVal[1].y*cvVal[2].y<0) || (cvVal[2].x*cvVal[3].x<0 && cvVal[2].y*cvVal[3].y<0) || (cvVal[3].x*cvVal[0].x<0 && cvVal[3].y*cvVal[0].y<0)){
            
            /*
            if(calcDegree(cvVal)!= 0){
                // append center to cp
                dvec2 cvCenter = (cvPos[0]+cvPos[2])/2.0;
                cp.push_back(cvCenter);
            }
             */

            // find v_x==0 and v_y==0
            std::vector<dvec2> pos_x;
            std::vector<dvec2> pos_y;
            dvec2 px, py, vx, vy;
            
            // find v_x == 0
            // (i, j) - (i, j+1)
            if(cvVal[0].x*cvVal[1].x<0){
                px = dvec2(cvPos[0].x+cellSize.x*(0-cvVal[0].x)/(cvVal[1].x-cvVal[0].x), cvPos[0].y);
                vx = vectorField.interpolate(px);
                if(-MINVAL<vx.x && vx.x < MINVAL){
                    pos_x.push_back(px);
                }
            }
            // (i, j) - (i+1, j)
            if(cvVal[1].x*cvVal[2].x<0){
                px = dvec2(cvPos[1].x ,cvPos[1].y+cellSize.y*(0-cvVal[1].x)/(cvVal[2].x-cvVal[1].x));
                vx = vectorField.interpolate(px);
                if(-MINVAL<vx.x && vx.x < MINVAL){
                    pos_x.push_back(px);
                }
            }
            // (i, j+1) - (i+1, j+1)
            if(cvVal[2].x*cvVal[3].x<0){
                px = dvec2(cvPos[2].x+cellSize.x*(0-cvVal[2].x)/(cvVal[3].x-cvVal[2].x), cvPos[2].y);
                vx = vectorField.interpolate(px);
                if(-MINVAL<vx.x && vx.x < MINVAL){
                    pos_x.push_back(px);
                }
            }
            // (i+1, j) - (i+1, j+1)
            if(cvVal[3].x*cvVal[0].x<0){
                px = dvec2(cvPos[3].x ,cvPos[3].y+cellSize.y*(0-cvVal[3].x)/(cvVal[0].x-cvVal[3].x));
                vx = vectorField.interpolate(px);
                if(-MINVAL<vx.x && vx.x < MINVAL){
                    pos_x.push_back(px);
                }
            }
            
            // find v_y == 0
            if(cvVal[0].y*cvVal[1].y<0){
                py = dvec2(cvPos[0].x+cellSize.x*(0-cvVal[0].y)/(cvVal[1].y-cvVal[0].y), cvPos[0].y);
                vy = vectorField.interpolate(py);
                if(-MINVAL<vy.y && vy.y < MINVAL){
                    pos_y.push_back(py);
                }
            }
            // (i, j) - (i+1, j)
            if(cvVal[1].x*cvVal[2].x<0){
                py = dvec2(cvPos[1].x ,cvPos[1].y+cellSize.y*(0-cvVal[1].y)/(cvVal[2].y-cvVal[1].y));
                vy = vectorField.interpolate(py);
                if(-MINVAL<vy.y && vy.y < MINVAL){
                    pos_y.push_back(py);
                }
            }
            // (i, j+1) - (i+1, j+1)
            if(cvVal[2].x*cvVal[3].x<0){
                py = dvec2(cvPos[2].x+cellSize.x*(0-cvVal[2].y)/(cvVal[3].y-cvVal[2].y), cvPos[2].y);
                vy = vectorField.interpolate(py);
                if(-MINVAL<vy.y && vy.y < MINVAL){
                    pos_y.push_back(py);
                }
            }
            // (i+1, j) - (i+1, j+1)
            if(cvVal[3].x*cvVal[0].x<0){
                py = dvec2(cvPos[3].x ,cvPos[3].y+cellSize.y*(0-cvVal[3].y)/(cvVal[0].y-cvVal[3].y));
                vy = vectorField.interpolate(py);
                if(-MINVAL<vy.y && vy.y < MINVAL){
                    pos_y.push_back(py);
                }
            }
            
            // if border point is cp
            for(auto iter=pos_x.begin(); iter!=pos_x.end(); iter++){
                dvec2 velocity = vectorField.interpolate(*iter);
                dmat2 jacobian = vectorField.derive(velocity);
                //cp.push_back(*iter); //打印看看位置
                //LogProcessorInfo("pos_x v:");
                //LogProcessorInfo(velocity);
                if(-MINVAL<velocity.x && velocity.x<MINVAL && -MINVAL<velocity.y && velocity.y<MINVAL && glm::determinant(jacobian) != 0){
                    cp.push_back(*iter);
                    pos_x.erase(iter);
                }
            }
            
            for(auto iter=pos_y.begin(); iter!=pos_y.end(); iter++){
                dvec2 velocity = vectorField.interpolate(*iter);
                dmat2 jacobian = vectorField.derive(velocity);
                //cp.push_back(*iter); //打印看看位置
                //LogProcessorInfo("pos_y v:");
                //LogProcessorInfo(velocity);
                if(-MINVAL<velocity.x && velocity.x<MINVAL && -MINVAL<velocity.y && velocity.y<MINVAL && glm::determinant(jacobian) != 0){
                    cp.push_back(*iter);
                    pos_y.erase(iter);
                }
            }
            
            // pos_x 和 pos_y 的 size 这里应该都是 2
            // 总之先打印一下
            
            LogProcessorInfo("pos_x:");
            for(auto iter = pos_x.begin(); iter!=pos_x.end(); iter++){
                LogProcessorInfo(*iter);
                cp.push_back(*iter);
            }
            LogProcessorInfo("pos_y:");
            for(auto iter = pos_y.begin(); iter!=pos_y.end(); iter++){
                LogProcessorInfo(*iter);
                cp.push_back(*iter);
            }
            
            
            
            if((int)pos_x.size() == 2 && (int)pos_y.size() == 2){
                dvec2 ip = getIntersectPoint(pos_x[0], pos_x[1], pos_y[0], pos_y[1]);
                LogProcessorInfo("ip:"<<ip);
                dvec2 velocity = vectorField.interpolate(ip);
                LogProcessorInfo("velocity:");
                LogProcessorInfo(velocity);
                dmat2 jacobian = vectorField.derive(velocity);
                LogProcessorInfo("jacobian determinant:");
                LogProcessorInfo(glm::determinant(jacobian));
                if(-MINVAL<velocity.x && velocity.x<MINVAL && -MINVAL<velocity.y && velocity.y<MINVAL && vectorField.isInside(ip) == true && glm::determinant(jacobian) != 0){
                    cp.push_back(ip);
                }
            }
            
        }
    }else{
        // domain descomposition
        if(scaleFactor>0){
            std::vector<dvec2> cvPosR(5, size2_t(0, 0));
            std::vector<dvec2> cvValR(5, size2_t(0, 0));
            // init.
            cvPosR[0] = (cvPos[0]+cvPos[1])/2.0; // up
            cvPosR[1] = (cvPos[1]+cvPos[2])/2.0; // right
            cvPosR[2] = (cvPos[2]+cvPos[3])/2.0; // bottom
            cvPosR[3] = (cvPos[3]+cvPos[0])/2.0; // left
            cvPosR[4] = (cvPos[0]+cvPos[2])/2.0; // center
            
            std::vector<dvec2> cvp, res;
            
            cvp = {cvPos[0], cvPosR[0], cvPosR[4], cvPosR[3]};
            res = extractCriticalPoints(vectorField, cvp);
            cp.insert(cp.end(), res.begin(), res.end());
            
            cvp = {cvPosR[0], cvPos[1], cvPosR[1], cvPosR[4]};
            res = extractCriticalPoints(vectorField, cvp);
            cp.insert(cp.end(), res.begin(), res.end());
            
            cvp = {cvPosR[4], cvPosR[1], cvPos[2], cvPosR[2]};
            res = extractCriticalPoints(vectorField, cvp);
            cp.insert(cp.end(), res.begin(), res.end());
            
            cvp = {cvPosR[3], cvPosR[4], cvPosR[2], cvPos[3]};
            res = extractCriticalPoints(vectorField, cvp);
            cp.insert(cp.end(), res.begin(), res.end());
        }
    }
    return cp;
    
    
}
