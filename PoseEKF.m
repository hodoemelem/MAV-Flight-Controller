%% with  wifi TCP/IP for control test
%%Make sure to connect PC to ESP wifi on you PC wifi connection.
%%Collect Data logged on the workspace after experiment
clc
device = tcpclient("192.168.4.1",1357);
X=0;Y=0;Z=0;
pX=0;pY=0;pZ=0;
str='';
sen=0;
j=1;
index=1;
started =0; 
xold=0;yold=0;zold=0;pxold=0;pyold=0;pzold=0;

while started == 0
    try  
             disp("waiting");
             sen=str2num(readline(device));
             xold=sen(1);yold=sen(2);zold=sen(3);pxold=sen(4);pyold=sen(5);pzold=sen(6);
             started =1;
              disp("Starting ..............");
    catch

    end
end;

X1=sen(1);
Y1=sen(2);
Z1=sen(3);
pX1=sen(4);
pY1=sen(5);
pZ1=sen(6);

%% For 2D plot
figureHandle = figure('NumberTitle','off',...
    'Name','EKF Postion Estimation',...
    'Color',[1 1 1],'Visible','on');

axesHandle = axes('Parent',figureHandle,...
    'YGrid','on',...
    'XGrid','on',...
    'YLim', [-45  45]);


        
index1=1;       
hold on;
plotHandle = plot(axesHandle,index1,X1,'r');
hold on;
plotHandle1 = plot(axesHandle,index1,Y1,'g');
hold on;
plotHandle2 = plot(axesHandle,index1,Z1,'b');
hold on;
plotHandle3 = plot(axesHandle,index1,pX1,'c');
hold on;
plotHandle4 = plot(axesHandle,index1,pY1,'k');
hold on;
plotHandle5 = plot(axesHandle,index1,pZ1,'y');
legend('X','Y','Z','pX/r','pY/p','pZ/y');
ylabel('XYZcm rpydeg ');
title('EKF Position Estimation');


%% For 3D plot
figureHandle1 = figure('NumberTitle','off',...
    'Name','EKF Postion Estimation',...
    'Color',[1 1 1],'Visible','on');

axesHandle1 = axes('Parent',figureHandle1);
hPlot = plot3(axesHandle1,sen(1), sen(2), sen(3),'.','Color','r','MarkerSize',12);
xlim([0 3087]);
ylim([0 4135]);
zlim([0 2354]);
xlabel('X')
ylabel('Y')
zlabel('z')
title('EKF Position Estimation')
grid on


%% Update plot values
while 1
  
       try  
            
            
             sen =str2num(readline(device));
             
             if numel(sen)== 6
                     X(j)=sen(1);
                     Y(j)=sen(2);
                     Z(j)=sen(3);
                     pX(j)=sen(4);
                     pY(j)=sen(5);
                     pZ(j)=sen(6);
                      xdata = [get(hPlot, 'XData') sen(1)];
                      ydata = [get(hPlot, 'YData') sen(2)]; 
                      zdata = [get(hPlot, 'ZData') sen(3)];
             xold=sen(1);yold=sen(2);zold=sen(3);pxold=sen(4);pyold=sen(5);pzold=0;%sen(6);
             disp(sen); 
             else
                    X(j)=xold;
                    Y(j)=yold;
                    Z(j)=zold;
                    pX(j)=pxold;
                    pY(j)=pyold;
                    pZ(j)=pzold;
                      xdata = [get(hPlot, 'XData') xold];
                      ydata = [get(hPlot, 'YData') yold]; 
                      zdata = [get(hPlot, 'ZData') zold];
             end  
   
            
             
        index(j)=j;
        if(j>500)
            index1=index(j-500:j);
            X1=X(j-500:j);
            Y1=Y(j-500:j);
            Z1=Z(j-500:j);
            pX1=pX(j-500:j);
            pY1=pY(j-500:j);
            pZ1=pZ(j-500:j);
           
            xmin=j-500;
            xmax=j;
        else
            index1=index;
            X1=X;
            Y1=Y;
            Z1=Z;
            pX1=pX;
            pY1=pY;
            pZ1=pZ;
            xmin=0;
            xmax=500;
        end
  
        set(hPlot, 'XData', xdata, 'YData', ydata, 'ZData', zdata);%for 3d plot
        set(plotHandle,'YData',X1,'XData',index1);
        set(plotHandle1,'YData',Y1,'XData',index1);
        set(plotHandle2,'YData',Z1,'XData',index1);
        set(plotHandle3,'YData',pX1,'XData',index1);
        set(plotHandle4,'YData',pY1,'XData',index1);
        set(plotHandle5,'YData',pZ1,'XData',index1);
        
        set(axesHandle,'XLim', [xmin xmax]);
        
        j=j+1;
        
        
        
       catch

       end
   
       
        
       
   

end;


%% with  wifi TCP/IP 
% %%Make sure to connect PC to ESP wifi on you PC wifi connection.
% %%Collect Data logged on the workspace after experiment
% clc
% device = tcpclient("192.168.4.1",1234);
% X=0;Y=0;Z=0;
% pX=0;pY=0;pZ=0;
% str='';
% sen=0;
% j=1;
% index=1;
% started =0; 
% xold=0;yold=0;zold=0;pxold=0;pyold=0;pzold=0;
% 
% while started == 0
%     try  
%              disp("waiting");
%              sen=str2num(readline(device));
%              xold=sen(1);yold=sen(2);zold=sen(3);pxold=sen(4);pyold=sen(5);pzold=sen(6);
%              started =1;
%               disp("Starting ..............");
%     catch
% 
%     end
% end;
% 
% X1=sen(1);
% Y1=sen(2);
% Z1=sen(3);
% pX1=sen(4);
% pY1=sen(5);
% pZ1=sen(6);
% 
% %% For 2D plot
% figureHandle = figure('NumberTitle','off',...
%     'Name','EKF Postion Estimation',...
%     'Color',[1 1 1],'Visible','on');
% 
% axesHandle = axes('Parent',figureHandle,...
%     'YGrid','on',...
%     'XGrid','on',...
%     'YLim', [-180  420]);
% 
% 
%         
% index1=1;       
% hold on;
% plotHandle = plot(axesHandle,index1,X1,'r');
% hold on;
% plotHandle1 = plot(axesHandle,index1,Y1,'g');
% hold on;
% plotHandle2 = plot(axesHandle,index1,Z1,'b');
% hold on;
% plotHandle3 = plot(axesHandle,index1,pX1,'c');
% hold on;
% plotHandle4 = plot(axesHandle,index1,pY1,'k');
% hold on;
% plotHandle5 = plot(axesHandle,index1,pZ1,'y');
% legend('X','Y','Z','pX/r','pY/p','pZ/y');
% ylabel('XYZcm rpydeg ');
% title('EKF Position Estimation');
% 
% 
% %% For 3D plot
% % figureHandle1 = figure('NumberTitle','off',...
% %     'Name','EKF Postion Estimation',...
% %     'Color',[1 1 1],'Visible','on');
% % 
% % axesHandle1 = axes('Parent',figureHandle1);
% % hPlot = plot3(axesHandle1,sen(1), sen(2), sen(3),'.','Color','r','MarkerSize',12);
% % xlim([0 3087]);
% % ylim([0 4135]);
% % zlim([0 2354]);
% % xlabel('X')
% % ylabel('Y')
% % zlabel('z')
% % title('EKF Position Estimation')
% % grid on
% 
% 
% %% Update plot values
% while 1
%   
%        try  
%             
%             
%              sen =str2num(readline(device));
%              
%              if numel(sen)== 6
%                      X(j)=sen(1)/10.0;
%                      Y(j)=sen(2)/10.0;
%                      Z(j)=sen(3)/10.0;
%                      pX(j)=sen(4);
%                      pY(j)=sen(5);
%                      pZ(j)=sen(6);
%         %              xdata = [get(hPlot, 'XData') sen(1)];
%         %              ydata = [get(hPlot, 'YData') sen(2)]; 
%         %              zdata = [get(hPlot, 'ZData') sen(3)];
%              xold=sen(1)/10.0;yold=sen(2)/10.0;zold=sen(3)/10.0;pxold=sen(4);pyold=sen(5);pzold=sen(6);
%              disp(sen); 
%              else
%                     X(j)=xold;
%                      Y(j)=yold;
%                      Z(j)=zold;
%                      pX(j)=pxold;
%                      pY(j)=pyold;
%                      pZ(j)=pzold;
%         %              xdata = [get(hPlot, 'XData') xold];
%         %              ydata = [get(hPlot, 'YData') yold]; 
%         %              zdata = [get(hPlot, 'ZData') zold];
%              end  
%    
%             
%              
%         index(j)=j;
%         if(j>500)
%             index1=index(j-500:j);
%             X1=X(j-500:j);
%             Y1=Y(j-500:j);
%             Z1=Z(j-500:j);
%             pX1=pX(j-500:j);
%             pY1=pY(j-500:j);
%             pZ1=pZ(j-500:j);
%            
%             xmin=j-500;
%             xmax=j;
%         else
%             index1=index;
%             X1=X;
%             Y1=Y;
%             Z1=Z;
%             pX1=pX;
%             pY1=pY;
%             pZ1=pZ;
%             xmin=0;
%             xmax=500;
%         end
%   
%         %set(hPlot, 'XData', xdata, 'YData', ydata, 'ZData', zdata);
%         set(plotHandle,'YData',X1,'XData',index1);
%         set(plotHandle1,'YData',Y1,'XData',index1);
%         set(plotHandle2,'YData',Z1,'XData',index1);
%         set(plotHandle3,'YData',pX1,'XData',index1);
%         set(plotHandle4,'YData',pY1,'XData',index1);
%         set(plotHandle5,'YData',pZ1,'XData',index1);
%         set(axesHandle,'XLim', [xmin xmax]);
%         
%         j=j+1;
%         
%         
%         
%        catch
% 
%        end
%    
%        
%         
%        
%    
% 
% end;














%% code For EFK position  Estimation plot using serial
% clc;
% clear device
% device=serialport("COM3",230400)
% X=0;Y=0;Z=0;
% pX=0;pY=0;pZ=0;
% str='';
% sen=0;
% j=1;
% index=1;
% 
% sen=str2num(readline(device));
% X1=sen(1);
% Y1=sen(2);
% Z1=sen(3);
% pX1=sen(4);
% pY1=sen(5);
% pZ1=sen(6);
% 
% %% For 2D plot
% figureHandle = figure('NumberTitle','off',...
%     'Name','EKF Postion Estimation',...
%     'Color',[1 1 1],'Visible','on');
% 
% axesHandle = axes('Parent',figureHandle,...
%     'YGrid','on',...
%     'XGrid','on',...
%     'YLim', [0  4135]);
% 
% 
%         
% index1=1;       
% hold on;
% plotHandle = plot(axesHandle,index1,X1,'r');
% hold on;
% plotHandle1 = plot(axesHandle,index1,Y1,'g');
% hold on;
% plotHandle2 = plot(axesHandle,index1,Z1,'b');
% hold on;
% plotHandle3 = plot(axesHandle,index1,pX1,'c');
% hold on;
% plotHandle4 = plot(axesHandle,index1,pY1,'k');
% hold on;
% plotHandle5 = plot(axesHandle,index1,pZ1,'y');
% legend('X','Y','Z','pX','pY','pZ');
% ylabel('mm');
% title('EKF Position Estimation');
% 
% 
% %% For 3D plot
% figureHandle1 = figure('NumberTitle','off',...
%     'Name','EKF Postion Estimation',...
%     'Color',[1 1 1],'Visible','on');
% 
% axesHandle1 = axes('Parent',figureHandle1);
% hPlot = plot3(axesHandle1,sen(1), sen(2), sen(3),'.','Color','r','MarkerSize',12);
% xlim([0 3087]);
% ylim([0 4135]);
% zlim([0 2354]);
% xlabel('X')
% ylabel('Y')
% zlabel('z')
% title('EKF Position Estimation')
% grid on
% 
% 
% %% Update plot values
% while 1
%   
%         sen=str2num(readline(device));
%         X(j)=sen(1);
%         Y(j)=sen(2);
%         Z(j)=sen(3);
%         pX(j)=sen(4);
%         pY(j)=sen(5);
%         pZ(j)=sen(6);
%   
%         xdata = [get(hPlot, 'XData') sen(1)];
%         ydata = [get(hPlot, 'YData') sen(2)]; 
%         zdata = [get(hPlot, 'ZData') sen(3)];
%         %zdata = [get(hPlot, 'ZData') 0];
%         index(j)=j;
%         if(j>500)
%             index1=index(j-500:j);
%             X1=X(j-500:j);
%             Y1=Y(j-500:j);
%             Z1=Z(j-500:j);
%             pX1=pX(j-500:j);
%             pY1=pY(j-500:j);
%             pZ1=pZ(j-500:j);
%            
%             xmin=j-500;
%             xmax=j;
%         else
%             index1=index;
%             X1=X;
%             Y1=Y;
%             Z1=Z;
%             pX1=pX;
%             pY1=pY;
%             pZ1=pZ;
%             xmin=0;
%             xmax=500;
%         end
%   
%         set(hPlot, 'XData', xdata, 'YData', ydata, 'ZData', zdata);
%         set(plotHandle,'YData',X1,'XData',index1);
%         set(plotHandle1,'YData',Y1,'XData',index1);
%         set(plotHandle2,'YData',Z1,'XData',index1);
%         set(plotHandle3,'YData',pX1,'XData',index1);
%         set(plotHandle4,'YData',pY1,'XData',index1);
%         set(plotHandle5,'YData',pZ1,'XData',index1);
%         set(axesHandle,'XLim', [xmin xmax]);
%         
%         j=j+1;
%    
% 
% end;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5

%% Serial plotter for Attitude and Heading
% clc;
% clear device
% device=serialport("COM3",115200)
% X=0;Y=0;Z=0;
% str='';
% sen=0;
% j=1;
% index=1;
% 
% sen=str2num(readline(device));
% X1=sen(1);
% Y1=sen(2);
% Z1=sen(3);
% pX1=sen(4);
% pY1=sen(5);
% pZ1=sen(6);
% 
% %% For 2D plot
% figureHandle = figure('NumberTitle','off',...
%     'Name','Attitude and Heading Plot',...
%     'Color',[1 1 1],'Visible','on');
% 
% axesHandle = axes('Parent',figureHandle,...
%     'YGrid','on',...
%     'XGrid','on',...
%     'YLim', [-180 360]);
% 
% 
%         
% index1=1;       
% hold on;
% plotHandle = plot(axesHandle,index1,X1,'r');
% hold on;
% plotHandle1 = plot(axesHandle,index1,Y1,'g');
% hold on;
% plotHandle2 = plot(axesHandle,index1,Z1,'b');
% hold on;
% plotHandle3 = plot(axesHandle,index1,pX1,'c');
% hold on;
% plotHandle4 = plot(axesHandle,index1,pY1,'k');
% hold on;
% plotHandle5 = plot(axesHandle,index1,pZ1,'m');
% legend('Roll','Pitch','Yaw','pozyxRoll','pozyxPitch','pozyxYaw');
% ylabel('Degrees');
% title('Attitude and Heading Plot');
% 
% 
% 
% 
% %% Update plot values
% while 1
%   
%         sen=str2num(readline(device));
%         X(j)=sen(1);
%         Y(j)=sen(2);
%         Z(j)=sen(3);
%         pX(j)=sen(4);
%         pY(j)=sen(5);
%         pZ(j)=sen(6);
% 
%         
%         index(j)=j;
%         if(j>500)
%             index1=index(j-500:j);
%             X1=X(j-500:j);
%             Y1=Y(j-500:j);
%             Z1=Z(j-500:j);
%             pX1=pX(j-500:j);
%             pY1=pY(j-500:j);
%             pZ1=pZ(j-500:j);
%            
%             xmin=j-500;
%             xmax=j;
%         else
%             index1=index;
%             X1=X;
%             Y1=Y;
%             Z1=Z;
%             pX1=pX;
%             pY1=pY;
%             pZ1=pZ;
%             xmin=0;
%             xmax=500;
%         end
%   
%       
%         set(plotHandle,'YData',X1,'XData',index1);
%         set(plotHandle1,'YData',Y1,'XData',index1);
%         set(plotHandle2,'YData',Z1,'XData',index1);
%         set(plotHandle3,'YData',pX1,'XData',index1);
%         set(plotHandle4,'YData',pY1,'XData',index1);
%         set(plotHandle5,'YData',pZ1,'XData',index1);
%         set(axesHandle,'XLim', [xmin xmax]);
%         
%         j=j+1;
%    
% 
% end;


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% %%%3D localization
% clc;
% 
% clear device
% device = serialport("COM3",115200)
% data = str2num(readline(device));
% hPlot = plot3(data(1), data(2), data(3),'.','Color','r','MarkerSize',12);
% xlim([0 7263]);
% ylim([0 4905]);
% zlim([0 7263]);
% 
% xlabel('X mm')
% ylabel('Y mm')
% zlabel('z mm')
% title('EKF Position Estimation')
% grid on
% 
% while 1
%     %pause(0.000005);
%     data = str2num(readline(device));
%     xdata = [get(hPlot, 'XData') data(1)];
%     ydata = [get(hPlot, 'YData') data(2)]; 
%     zdata = [get(hPlot, 'ZData') data(3)];
%     set(hPlot, 'XData', xdata, 'YData', ydata, 'ZData', zdata);
%     
% end


