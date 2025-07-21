const ip = '192.168.31.248:'
const port = '9090'; // 替换为你的Rosbridge服务器端口
async function loadConfig() {
    try {
        const response = await fetch('config.json');
        if (!response.ok) throw new Error('配置文件加载失败');
        const config_data = await response.json();
        console.log('配置加载成功:', config_data);
        if (!config_data.rosbridge_ip || !config_data.rosbridge_port || !config_data.http_port) {
            throw new Error('配置文件缺少必要字段');
        }
        return config_data;
    } catch (error) {
        console.error('配置加载错误:', error);
        // 提供默认配置作为回退
        return {
            rosbridge_ip: "192.168.31.248",
            rosbridge_port: "9090",
            http_port: 8080,
            _isFallBack: true
        };
    }
}

// 全局变量
let verticalValue = 0.5;
let verticalActive = false;
let verticalStartY = 0;
let verticalSliderHeight = 200;

// 摇杆输出值
let leftOutput = { x: 0, y: 0 };
let rightOutput = { x: 0 };
let heightOutput = 0.5;

// 活动触摸点记录
let activeTouches = {
    left: null,
    vertical: null,
    right: null,
    pitch: null
};

// 高度滑块控制元素
const verticalKnob = document.getElementById('vertical-knob');
const verticalSlider = document.getElementById('vertical-slider');
const fillIndicator = document.getElementById('fill-indicator');
const pitchKnob = document.getElementById('pitch-knob');
const pitchSlider = document.getElementById('pitch-slider');
const fillIndicator2 = document.getElementById('fill-indicator2');

// 摇杆元素
const leftJoystickOuter = document.getElementById('left-outer');
const leftJoystickInner = document.getElementById('left-inner');
const rightJoystickOuter = document.getElementById('right-outer');
const rightJoystickInner = document.getElementById('right-inner');
// 按钮元素
const resetButton = document.getElementById('reset-all');

// 新版更新调试信息函数
// 在全局作用域定义消息管理对象
const messageRegistry = new Map();
// 重写消息显示函数
function updateDebugInfo(id, message) {
    const debugList = document.getElementById('debugList');
    const timestamp = `[${new Date().toLocaleTimeString()}]`;
    let msgElement = document.querySelector(`[data-id="${id}"]`); // 通过ID查找元素
    // 情况1：元素存在且已隐藏 → 重激活
    if (msgElement && msgElement.style.display === 'none') {
        msgElement.textContent = `${timestamp} ${message}`;
        msgElement.style.opacity = '1';
        msgElement.style.display = 'block'; // 重新显示
        clearTimeout(msgElement.timer); // 清除旧计时器
        msgElement.timer = startFadeTimer(msgElement, id); // 重启淡出计时
        messageRegistry.set(id, msgElement); // 重新注册
        return;
    }
    // 情况2：元素存在且显示中 → 更新内容并重置计时器
    if (msgElement) {
        msgElement.textContent = `${timestamp} ${message}`;
        clearTimeout(msgElement.timer);
        msgElement.timer = startFadeTimer(msgElement, id);
        return;
    }
    // 情况3：元素不存在 → 新建消息
    msgElement = document.createElement('div');
    msgElement.className = 'debug-item';
    msgElement.dataset.id = id; // 标记ID以便查找
    msgElement.textContent = `${timestamp} ${message}`;
    debugList.prepend(msgElement);
    msgElement.timer = startFadeTimer(msgElement, id); // 启动淡出计时
    messageRegistry.set(id, msgElement); // 注册新消息
    manageMessageQueue(); // 管理队列长度
    debugList.scrollTop = 0;
}
// 启动淡出计时器
function startFadeTimer(element, id) {
    return setTimeout(() => {
        element.style.transition = 'opacity 0.5s ease';
        element.style.opacity = '0';
        // 淡出后隐藏而非移除元素
        setTimeout(() => {
            element.style.display = 'none'; 
            messageRegistry.delete(id); // 清理注册表记录
        }, 500);
    }, 3000);
}
// 管理消息队列
function manageMessageQueue() {
    const messages = debugList.querySelectorAll('.debug-item');
    if (messages.length > 20) {
        // 移除最旧且未显示的消息（非激活状态）
        const oldestMsg = Array.from(messages).find(msg => 
            msg.style.display !== 'none'
        );
        oldestMsg?.remove();
    }
}

// 初始化高度滑块位置
function initVerticalSlider() {
    const sliderRect = verticalSlider.getBoundingClientRect();
    verticalSliderHeight = sliderRect.height;
    const knobHeight = verticalKnob.offsetHeight;
    const maxY = verticalSliderHeight - knobHeight;
    const currentY = maxY * (1 - verticalValue);
    
    verticalKnob.style.top = currentY + 'px';
    fillIndicator.style.height = (verticalValue * 100) + '%';
    
    updateDebugInfo('heigh-init',`高度控制初始化完成: 总高=${verticalSliderHeight}px, 滑块位置=${currentY}px`);
}

// 高度条事件处理 - 支持多点触控
function setupVerticalSliderEvents() {
    // 滑块触摸开始
    verticalKnob.addEventListener('touchstart', function(e) {
        // 检查是否已有一个活动触摸点
        if (activeTouches.vertical !== null) return;
        
        const touch = e.changedTouches[0];
        verticalActive = true;
        activeTouches.vertical = touch.identifier;
        
        // 记录初始触摸位置
        const knobRect = verticalKnob.getBoundingClientRect();
        verticalStartY = touch.clientY - knobRect.top;
        
        // 添加活动状态样式
        verticalKnob.style.transform = 'translateX(-50%) scale(1.2)';
        verticalSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("height-info","高度滑块开始触摸");
    }, { passive: false });
    
    // 滑块背景触摸
    verticalSlider.addEventListener('touchstart', function(e) {
        if (activeTouches.vertical !== null) return;
        
        const touch = e.changedTouches[0];
        verticalActive = true;
        activeTouches.vertical = touch.identifier;
        
        const sliderRect = verticalSlider.getBoundingClientRect();
        // 计算新位置
        const newY = touch.clientY - sliderRect.top - (verticalKnob.offsetHeight / 2);
        
        // 更新高度值
        updateVerticalValue(newY);
        
        // 添加活动状态样式
        verticalKnob.style.transform = 'translateX(-50%) scale(1.2)';
        verticalSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("height-info","高度条触摸开始");
    }, { passive: false });
    
    // 滑块鼠标按下
    verticalKnob.addEventListener('mousedown', function(e) {
        verticalActive = true;
        const knobRect = verticalKnob.getBoundingClientRect();
        verticalStartY = e.clientY - knobRect.top;
        
        verticalKnob.style.transform = 'translateX(-50%) scale(1.2)';
        verticalSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("height-info","高度滑块鼠标按下");
    });
    
    verticalSlider.addEventListener('mousedown', function(e) {
        if (e.target === verticalSlider || e.target === fillIndicator) {
            verticalActive = true;
            const sliderRect = verticalSlider.getBoundingClientRect();
            const newY = e.clientY - sliderRect.top - (verticalKnob.offsetHeight / 2);
            
            updateVerticalValue(newY);
            
            verticalKnob.style.transform = 'translateX(-50%) scale(1.2)';
            verticalSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
            
            e.preventDefault();
            updateDebugInfo("height-info","高度条鼠标按下");
        }
    });
}

// 更新高度值
function updateVerticalValue(yPos) {
    const knobHeight = verticalKnob.offsetHeight;
    const minY = 0;
    const maxY = verticalSliderHeight - knobHeight;
    
    // 约束在有效范围内
    let constrainedY = Math.max(minY, Math.min(maxY, yPos));
    
    // 计算高度值 (0-1)
    const newValue = 1 - (constrainedY / maxY);
    verticalValue = newValue;
    
    // 更新滑块位置
    verticalKnob.style.top = constrainedY + 'px';
    
    // 更新填充指示器
    fillIndicator.style.height = (newValue * 100) + '%';
    
    // 更新高度显示值
    document.getElementById('vertical-value').textContent = verticalValue.toFixed(2);
    heightOutput = verticalValue; // 更新高度输出值
    
    // 调试信息
    updateDebugInfo("height-info",`高度值: ${verticalValue.toFixed(2)}, Y位置: ${constrainedY.toFixed(0)}px`);
}
let pitchValue = 0.5;
let pitchActive = false;
let pitchStartY = 0;
let pitchSliderHeight = 200;
// 初始化倾角滑块
function initPitchSlider() {
    const sliderRect = pitchSlider.getBoundingClientRect();
    pitchSliderHeight = sliderRect.height;
    const knobHeight = pitchKnob.offsetHeight;
    const maxY = pitchSliderHeight - knobHeight;
    const currentY = maxY * (1 - pitchValue);
    
    pitchKnob.style.top = currentY + 'px';
    fillIndicator2.style.height = (pitchValue * 100) + '%';
    
    updateDebugInfo("pitch-init", `倾角控制初始化完成: 总高=${pitchSliderHeight}px, 滑块位置=${currentY}px`);
}
// 更新倾角值
function updatePitchValue(yPos) {
    const knobHeight = pitchKnob.offsetHeight;
    const minY = 0;
    const maxY = pitchSliderHeight - knobHeight;
    
    let constrainedY = Math.max(minY, Math.min(maxY, yPos));
    const newValue = 1 - (constrainedY / maxY);
    pitchValue = newValue;
    
    pitchKnob.style.top = constrainedY + 'px';
    fillIndicator2.style.height = (newValue * 100) + '%';
    
    // 更新显示值
    const pitchDisplay = document.getElementById('pitch-value');
    if(pitchDisplay) {
        pitchDisplay.textContent = pitchValue.toFixed(2);
    }
    
    updateDebugInfo("pitch-info", `倾角值: ${pitchValue.toFixed(2)}, Y位置: ${constrainedY.toFixed(0)}px`);
}
// 设置倾角滑块事件
function setupPitchSliderEvents() {
    // 滑块触摸开始
    pitchKnob.addEventListener('touchstart', function(e) {
        if (activeTouches.pitch !== null) return;
        const touch = e.changedTouches[0];
        pitchActive = true;
        activeTouches.pitch = touch.identifier;
        
        const knobRect = pitchKnob.getBoundingClientRect();
        pitchStartY = touch.clientY - knobRect.top;
        
        pitchKnob.style.transform = 'translateX(-50%) scale(1.2)';
        pitchSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("pitch-info", "倾角滑块开始触摸");
    }, { passive: false });
    
    // 背景触摸
    pitchSlider.addEventListener('touchstart', function(e) {
        if (activeTouches.pitch !== null) return;
        const touch = e.changedTouches[0];
        pitchActive = true;
        activeTouches.pitch = touch.identifier;
        
        const sliderRect = pitchSlider.getBoundingClientRect();
        const newY = touch.clientY - sliderRect.top - (pitchKnob.offsetHeight / 2);
        
        updatePitchValue(newY);
        
        pitchKnob.style.transform = 'translateX(-50%) scale(1.2)';
        pitchSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("pitch-info", "倾角条触摸开始");
    }, { passive: false });
    
    // 鼠标事件
    pitchKnob.addEventListener('mousedown', function(e) {
        pitchActive = true;
        const knobRect = pitchKnob.getBoundingClientRect();
        pitchStartY = e.clientY - knobRect.top;
        
        pitchKnob.style.transform = 'translateX(-50%) scale(1.2)';
        pitchSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
        
        e.preventDefault();
        updateDebugInfo("pitch-info", "倾角滑块鼠标按下");
    });
    
    pitchSlider.addEventListener('mousedown', function(e) {
        if (e.target === pitchSlider || e.target === fillIndicator2) {
            pitchActive = true;
            const sliderRect = pitchSlider.getBoundingClientRect();
            const newY = e.clientY - sliderRect.top - (pitchKnob.offsetHeight / 2);
            
            updatePitchValue(newY);
            
            pitchKnob.style.transform = 'translateX(-50%) scale(1.2)';
            pitchSlider.style.boxShadow = '0 0 20px rgba(255, 85, 0, 0.5)';
            
            e.preventDefault();
            updateDebugInfo("pitch-info", "倾角条鼠标按下");
        }
    });
}

// 摇杆功能 - 支持多点触控
function setupJoystickEvents() {
    // 左侧摇杆触摸开始
    leftJoystickOuter.addEventListener('touchstart', function(e) {
        // 检查是否已有一个活动触摸点
        if (activeTouches.left !== null) return;
        
        const touch = e.changedTouches[0];
        activeTouches.left = touch.identifier;
        
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        // 更新摇杆位置
        updateJoystickPosition('left', touch.clientX - centerX, touch.clientY - centerY);
        
        e.preventDefault();
        updateDebugInfo("left-joystick","左摇杆开始触摸");
    }, { passive: false });
    
    // 右侧摇杆触摸开始
    rightJoystickOuter.addEventListener('touchstart', function(e) {
        // 检查是否已有一个活动触摸点
        if (activeTouches.right !== null) return;
        
        const touch = e.changedTouches[0];
        activeTouches.right = touch.identifier;
        
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        // 更新摇杆位置
        updateJoystickPosition('right', touch.clientX - centerX, touch.clientY - centerY);
        
        e.preventDefault();
        updateDebugInfo("right-joystick","右摇杆开始触摸");
    }, { passive: false });
    
    // 左侧摇杆鼠标按下
    leftJoystickOuter.addEventListener('mousedown', function(e) {
        activeTouches.left = 'mouse';
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    });
    
    // 右侧摇杆鼠标按下
    rightJoystickOuter.addEventListener('mousedown', function(e) {
        activeTouches.right = 'mouse';
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        
        updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    });

    // 新增鼠标移动事件绑定
    leftJoystickOuter.addEventListener('mousemove', function(e) {
        if (activeTouches.left === 'mouse') {
            const rect = leftJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        }
    });

    rightJoystickOuter.addEventListener('mousemove', function(e) {
        if (activeTouches.right === 'mouse') {
            const rect = rightJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        }
    });
}

// 更新摇杆位置
function updateJoystickPosition(type, dx, dy) {
    const rect = (type === 'left' ? 
                  leftJoystickOuter : 
                  rightJoystickOuter).getBoundingClientRect();
    
    // 最大移动距离
    const maxDistance = Math.min(rect.width, rect.height) * 0.35;
    
    const distance = Math.sqrt(dx * dx + dy * dy);
    let newDx = dx;
    let newDy = dy;
    
    // 限制在最大距离内
    if (distance > maxDistance) {
        const scale = maxDistance / distance;
        newDx = dx * scale;
        newDy = dy * scale;
    }
    
    // 更新摇杆位置
    const element = (type === 'left' ? 
                     leftJoystickInner : 
                     rightJoystickInner);
    element.style.transform = `translate(calc(-50% + ${newDx}px), calc(-50% + ${newDy}px))`;
    
    // 存储输出值
    if (type === 'left') {
        leftOutput.x = newDx / maxDistance;
        leftOutput.y = newDy / maxDistance;
        if (leftOutput.y * leftOutput.y + leftOutput.x * leftOutput.x < 0.1) {
            leftOutput = { x: 0, y: 0 }; // 重置为0
        }
        document.getElementById('left-x-output').textContent = (-leftOutput.y).toFixed(2);
        document.getElementById('left-y-output').textContent = leftOutput.x.toFixed(2);
        updateDebugInfo("left-info",`左摇杆: X:${-leftOutput.y.toFixed(2)}, Y:${leftOutput.x.toFixed(2)}`);
    } else {
        rightOutput.x = newDx / maxDistance;
        if (rightOutput.x * rightOutput.x < 0.1) {
            rightOutput = { x: 0 }; // 重置为0
        }
        document.getElementById('right-x-output').textContent = rightOutput.x.toFixed(2);
        updateDebugInfo("right-info",`右摇杆: X:${rightOutput.x.toFixed(2)}`);
    }
}

// 触摸移动处理 - 多点触控支持
document.addEventListener('touchmove', function(e) {
    e.preventDefault();
    
    for (let i = 0; i < e.touches.length; i++) {
        const touch = e.touches[i];
        
        // 更新左侧摇杆
        if (activeTouches.left === touch.identifier) {
            const rect = leftJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            
            updateJoystickPosition('left', touch.clientX - centerX, touch.clientY - centerY);
        }
        
        // 更新右侧摇杆
        if (activeTouches.right === touch.identifier) {
            const rect = rightJoystickOuter.getBoundingClientRect();
            const centerX = rect.left + rect.width/2;
            const centerY = rect.top + rect.height/2;
            
            updateJoystickPosition('right', touch.clientX - centerX, touch.clientY - centerY);
        }
        
        // 更新高度条
        if (activeTouches.vertical === touch.identifier) {
            const sliderRect = verticalSlider.getBoundingClientRect();
            
            // 计算新位置
            let newY;
            if (touch.target === verticalKnob) {
                newY = touch.clientY - sliderRect.top - verticalStartY;
            } else {
                newY = touch.clientY - sliderRect.top - (verticalKnob.offsetHeight / 2);
            }
            
            // 更新高度值
            updateVerticalValue(newY);
        }
        // 更新倾角条
        if (activeTouches.pitch === touch.identifier) {
            const sliderRect = pitchSlider.getBoundingClientRect();
            let newY;
            if (touch.target === pitchKnob) {
                newY = touch.clientY - sliderRect.top - pitchStartY;
            } else {
                newY = touch.clientY - sliderRect.top - (pitchKnob.offsetHeight / 2);
            }
            updatePitchValue(newY);
        }
    }
}, { passive: false });

// 触摸结束处理
document.addEventListener('touchend', function(e) {
    for (let i = 0; i < e.changedTouches.length; i++) {
        const touch = e.changedTouches[i];
        
        // 重置左侧摇杆
        if (activeTouches.left === touch.identifier) {
            resetJoystick('left');
            activeTouches.left = null;
            updateDebugInfo("left-joystick","左摇杆结束触摸");
        }
        
        // 重置右侧摇杆
        if (activeTouches.right === touch.identifier) {
            resetJoystick('right');
            activeTouches.right = null;
            updateDebugInfo("right-joystick","右摇杆结束触摸");
        }
        
        // 重置高度条
        if (activeTouches.vertical === touch.identifier) {
            verticalActive = false;
            activeTouches.vertical = null;
            
            // 移除活动状态样式
            verticalKnob.style.transform = 'translateX(-50%) scale(1)';
            verticalSlider.style.boxShadow = '';
            updateDebugInfo("height-info","高度滑块结束触摸");
        }
        // 重置倾角条
        if (activeTouches.pitch === touch.identifier) {
            pitchActive = false;
            activeTouches.pitch = null;
            pitchKnob.style.transform = 'translateX(-50%) scale(1)';
            pitchSlider.style.boxShadow = '';
            updateDebugInfo("pitch-info", "倾角滑块结束触摸");
        }
    }
});

// 鼠标移动处理
document.addEventListener('mousemove', function(e) {
    if (verticalActive) {
        const sliderRect = verticalSlider.getBoundingClientRect();
        const newY = e.clientY - sliderRect.top - verticalStartY;
        updateVerticalValue(newY);
    }
    
    if (activeTouches.left === 'mouse') {
        const rect = leftJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        updateJoystickPosition('left', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    }
    
    if (activeTouches.right === 'mouse') {
        const rect = rightJoystickOuter.getBoundingClientRect();
        const centerX = rect.left + rect.width/2;
        const centerY = rect.top + rect.height/2;
        updateJoystickPosition('right', e.clientX - centerX, e.clientY - centerY);
        e.preventDefault();
    }
    if (pitchActive) {
        const sliderRect = pitchSlider.getBoundingClientRect();
        const newY = e.clientY - sliderRect.top - pitchStartY;
        updatePitchValue(newY);
    }
});

// 鼠标抬起处理
document.addEventListener('mouseup', function() {
    if (verticalActive) {
        verticalActive = false;
        verticalKnob.style.transform = 'translateX(-50%) scale(1)';
        verticalSlider.style.boxShadow = '';
        updateDebugInfo("height-info","高度滑块鼠标抬起");
    }
    
    if (activeTouches.left === 'mouse') {
        resetJoystick('left');
        activeTouches.left = null;
    }
    
    if (activeTouches.right === 'mouse') {
        resetJoystick('right');
        activeTouches.right = null;
    }
    if (pitchActive) {
        pitchActive = false;
        pitchKnob.style.transform = 'translateX(-50%) scale(1)';
        pitchSlider.style.boxShadow = '';
        updateDebugInfo("pitch-info", "倾角滑块鼠标抬起");
    }
});

// 重置摇杆位置
function resetJoystick(type) {
    const element = (type === 'left' ? 
                     leftJoystickInner : 
                     rightJoystickInner);
    element.style.transform = 'translate(-50%, -50%)';
    
    if (type === 'left') {
        leftOutput = { x: 0, y: 0 };
        document.getElementById('left-x-output').textContent = "0.00";
        document.getElementById('left-y-output').textContent = "0.00";
    } else {
        rightOutput = { x: 0 };
        document.getElementById('right-x-output').textContent = "0.00";
    }
}
let config = {
    rosbridge_ip: "192.168.31.248",
    rosbridge_port: "9090",
    http_port: 8080,
    publish_interval: 10,
    _isFallBack: false,
}
let jumpTopic; // 起跳话题对象
let jumpInterval; // 起跳定时器

let flipTopic; // 翻转话题对象
let flipInterval; // 翻转定时器
// 重置所有控制器
function resetAllControls() {
    // 重置摇杆
    resetJoystick('left');
    resetJoystick('right');
    
    // 重置高度滑块
    verticalValue = 0.5;
    const knobHeight = verticalKnob.offsetHeight;
    const maxY = verticalSliderHeight - knobHeight;
    verticalKnob.style.top = maxY * 0.5 + 'px';
    fillIndicator.style.height = '50%';
    document.getElementById('vertical-value').textContent = '0.50';
    
    // 重置倾角滑块
    pitchValue = 0.5;
    pitchKnob.style.top = maxY * 0.5 + 'px';
    fillIndicator2.style.height = '50%';
    document.getElementById('pitch-value').textContent = '0.50';
    
    // 重置输出值
    heightOutput = 0.5;
    leftOutput = { x: 0, y: 0 };
    rightOutput = { x: 0 };
    
    updateDebugInfo("sys-reset", "所有控制器已重置");
}
// 触发机器人起跳
function triggerJump() {
    if (!jumpTopic || !connected) {
        updateDebugInfo("jump-error", "未连接到ROS或jump主题未初始化");
        return;
    }
    
    let count = 0;
    const maxCount = 1;
    const intervalTime = 10; // 10ms * 10 = 100ms
    
    clearInterval(jumpInterval); // 清除现有定时器
    jumpInterval = setInterval(() => {
        if (count >= maxCount) {
            clearInterval(jumpInterval);
            return;
        }
        
        const jumpMsg = new ROSLIB.Message({
            data: true
        });
        
        jumpTopic.publish(jumpMsg);
        document.getElementById('message-count').textContent = 
            parseInt(document.getElementById('message-count').textContent) + 1;
        
        updateDebugInfo("jump-publish", `发送起跳指令 ${count+1}/${maxCount}`);
        count++;
    }, intervalTime);
}

// 触发机器人起跳
function triggerFlip() {
    if (!flipTopic || !connected) {
        updateDebugInfo("flip-error", "未连接到ROS或flip主题未初始化");
        return;
    }
    
    let count = 0;
    const maxCount = 1;
    const intervalTime = 10; // 10ms * 10 = 100ms
    
    clearInterval(flipInterval); // 清除现有定时器
    flipInterval = setInterval(() => {
        if (count >= maxCount) {
            clearInterval(flipInterval);
            return;
        }
        
        const flipMsg = new ROSLIB.Message({
            data: true
        });
        
        flipTopic.publish(flipMsg);
        document.getElementById('message-count').textContent = 
            parseInt(document.getElementById('message-count').textContent) + 1;
        
        updateDebugInfo("flip-publish", `发送起跳指令 ${count+1}/${maxCount}`);
        count++;
    }, intervalTime);
}

// 页面加载完成后初始化
window.addEventListener('DOMContentLoaded', async () => {
    const LoadedConfig = await loadConfig();
    if (LoadedConfig._isFallBack) {
        updateDebugInfo("ros-config",'使用默认配置: ROS ' + config.rosbridge_ip + ':' + config.rosbridge_port + ', HTTP端口 ' + config.http_port + ', 发布频率 ' + config.publish_interval + 'ms');
    } else {
        updateDebugInfo("ros-config",'加载配置成功: ROS ' + config.rosbridge_ip + ':' + config.rosbridge_port + ', HTTP端口 ' + config.http_port + ', 发布频率 ' + config.publish_interval + 'ms');
    }
    config = LoadedConfig; // 更新全局配置
    // 初始化高度滑块
    initVerticalSlider();
    initPitchSlider();
    
    // 设置事件监听
    setupVerticalSliderEvents();
    setupJoystickEvents();
    setupPitchSliderEvents();
    
    // 按钮事件
    document.getElementById('connectBtn').addEventListener('click', function() {
        startConnection();
        updateDebugInfo("ros-connection",'尝试连接到ROS服务器...');
    });
    
    document.getElementById('disconnectBtn').addEventListener('click', function() {
        if (ros) {
            ros.close();
            updateDebugInfo("ros-connection",'已断开与ROS服务器的连接');
            document.getElementById('connectBtn').textContent = '连接 ROS';
            document.getElementById('disconnectBtn').textContent = '断开连接';
            document.getElementById('ros-status').textContent = '未连接';
            document.getElementById('ros-indicator').className = 'indicator disconnected';
            connected = false;
        } else {
            updateDebugInfo("ros-connection",'当前未连接到ROS服务器');
        }
    });
    
    // 初始状态
    updateDebugInfo("sys-info",'系统初始化完成');
    
    // 窗口大小变化时重新计算位置
    window.addEventListener('resize', function() {
        initVerticalSlider();
        initPitchSlider();
        updateDebugInfo("sys-info",'重新计算所有控制位置');
    });
    // 在初始化部分添加防抖的resize处理
    let resizeTimer;
    window.addEventListener('resize', function() {
        clearTimeout(resizeTimer);
        resizeTimer = setTimeout(() => {
            initVerticalSlider();
            updateDebugInfo("sys-info",'窗口尺寸更新完成');
        }, 500); // 500ms防抖
    });
    // 添加重置按钮事件
    document.getElementById('reset-all').addEventListener('click', resetAllControls);
    
    // 添加上升按钮事件（第二个按钮改为起跳功能）
    const jumpBtn = document.getElementById('jump-btn');
    jumpBtn.addEventListener('click', triggerJump);

    const flipBtn = document.getElementById('flip-btn');
    flipBtn.addEventListener('click', triggerFlip);
});
let ros = null;
let cmdVel = null;
let height = null;
let connected = false;

function startConnection() {
    if (ros) {
        ros.close();
        ros = null;
    }
    ros = new ROSLIB.Ros({
        url: `ws://${config.rosbridge_ip}:${config.rosbridge_port}`
    });
    ros.on('connection', () => {
        console.log('Connected to ROS Bridge');
        cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
        });
        height = new ROSLIB.Topic({
            ros: ros,
            name: '/height',
            messageType: 'std_msgs/Float32'
        });
        jumpTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/jump',
            messageType: 'std_msgs/Bool'
        });
        flipTopic = new ROSLIB.Topic({
            ros: ros,
            name: '/flip',
            messageType: 'std_msgs/Bool'
        });
        updateDebugInfo("ros-connection",'已连接到ROS桥接');
        document.getElementById('connectBtn').textContent = '已连接';
        document.getElementById('disconnectBtn').textContent = '断开连接';
        document.getElementById('ros-status').textContent = '已连接';
        document.getElementById('ros-indicator').className = 'indicator connected';
        connected = true;
        jumpTopic.publish(new ROSLIB.Message({ data: false })); // 初始化起跳话题
        flipTopic.publish(new ROSLIB.Message({ data: false })); // 初始化翻转话题
    });
    ros.on('error', (error) => {
        console.error('Error connecting to ROS: ', error);
        alert('Error connecting to ROS: 9090', error);
    });
    ros.on('close', () => {
        console.log('Disconnected from ROS');
        updateDebugInfo("ros-connection",'已断开与ROS服务器的连接');
    });
}
function move(){
    if (!ros) {
        console.error('ROS connection not established');
        alert('ROS connection not established');
        return;
    }
    if (!cmdVel) {
        console.error('Publisher not created');
        alert('Publisher not created');
        return;
    }
    if (!height) {
        console.error('Height publisher not created');
        alert('Height publisher not created');
        return;
    }
    const twist = new ROSLIB.Message({
        linear: {
            x: -leftOutput.y,
            y: leftOutput.x,
            z: 0
        },
        angular: {
            x: 0,
            y: pitchValue, // 假设高度控制影响y轴
            z: -rightOutput.x // 假设右摇杆控制角速度
        }
    });
    const heightMsg = new ROSLIB.Message({
        data: heightOutput // 使用高度输出值
    });
    // 发布消息
    console.log(`Publishing: linear x=${twist.linear.x}, y=${twist.linear.y}, angular y=${twist.angular.y}, z=${twist.angular.z}`);
    cmdVel.publish(twist);
    height.publish(heightMsg);
    document.getElementById('message-count').textContent = parseInt(document.getElementById('message-count').textContent) + 1;
}
// 定时发布消息
const interval = setInterval(() => {
    if (connected) {
        move();
    }
}, config.publish_interval);