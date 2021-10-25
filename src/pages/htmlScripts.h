// Javascript for Ajax
#pragma once

#include "../Common.h"

const char html_ajax_activeA[] PROGMEM =
"<script>\n"
"var auto1Tick=0;\n"
"var auto1=setInterval(autoRun1s," STR(RESPONSE_INTERVAL) ");\n"

"function autoRun1s() {\n"
  "var pageList = ["
  "'MiscStatus',15,"
  #if WEATHER == ON
    "'Weather',32,"
  #endif
  #if THERMOSTAT == ON
    "'ThermostatT',61,"
      #if THERMOSTAT_HUMIDITY == ON
        "'ThermostatH',118,"
      #endif
  #endif
  #if ROOF == ON
    "'RoofStatus',3,"
  #endif
  "'',0"
  "];\n"

  "auto1Tick++;\n"
  "var i;\n"
  "var request;\n"
  "for (i=0; i<pageList.length-1; i+=2) {\n"
    "if (auto1Tick%pageList[i+1]==0) {\n"
      "thisPage=pageList[i];\n"
      "nocache='?nocache='+Math.random()*1000000;\n";
const char html_ajax_activeB[] PROGMEM =
      "request = new XMLHttpRequest();\n"
      "request.onreadystatechange = pageReady(thisPage);\n"
      "request.open('GET',thisPage.toLowerCase()+nocache,true); request.send(null);\n"
    "}"
  "}"
"}\n"
"function pageReady(aPage) {\n"
  "return function() {\n"
    "if ((this.readyState==4)&&(this.status==200)) {\n"
      "document.getElementById(aPage).innerHTML=this.responseText;\n"
    "}"
  "}"
"}\n"
"</script>\n";

const char html_ajax_setRelay[] PROGMEM = 
"<script>\n"
"function SetRelay(relay,value) {\n"
  "nocache = '&nocache=' + Math.random() * 1000000;\n"
  "var request = new XMLHttpRequest();\n"
  "request.open('GET','relay?r'+relay+'='+value+nocache,true);\n"
  "request.send(null);\n"
"}\n"
"</script>\n";

const char html_ajax_setVar[] PROGMEM = 
"<script>\n"
"function SetVar(name,value) {\n"
  "nocache = '&nocache=' + Math.random() * 1000000;\n"
  "var request = new XMLHttpRequest();\n"
  "request.open('GET','setvar?'+name+'='+value+nocache,true);\n"
  "request.send(null);\n"
"}\n"
"</script>\n";

const char ChartJs1[] PROGMEM =
"ctx%s=document.getElementById(\"%s\");"
"var scatterChart = new Chart(ctx%s, {"
"type: 'line',"
  "data: {"
    "datasets: [{"
      "label: '";
const char ChartJs3[] PROGMEM = "',"
      "backgroundColor: \"rgba(192,32,32,0.4)\","
      "data: [";
const char ChartJs4[] PROGMEM = 
      "]"
    "}]"
  "},"
  "options: {"
    "scales: {"
      "xAxes: [{"
        "type: 'linear',"
        "position: 'bottom'"
      "}],"
      "yAxes: [{"
        "ticks: {"
          "max: %d,"
          "min: %d,"
          "stepSize: %d"
        "}"
      "}]"
    "}"
  "}"
"});\n";

// Javascript for Ajax return
const char html_ajaxScript[] PROGMEM =
"<script>\n"
"function s(key,v1) {"
  "var xhttp = new XMLHttpRequest();"
  "xhttp.open('GET','%s?'+key+'='+encodeURIComponent(v1)+'&x='+new Date().getTime(), true);"
  "xhttp.send();"
"}</script>\n";

const char html_ajaxScriptShort[] PROGMEM =
"<script>\n"
"function g(v1){s('dr',v1);}"
"function gf(v1){s('dr',v1);autoFastRun();}"
"function sf(key,v1){s(key,v1);autoFastRun();}\n"
"</script>\n";

// Javascript for Date/Time return
const char html_dateTimeScriptA[] PROGMEM =
"<script>\r\n"
"function SetDateTime() {"
"var d1 = new Date();"
"var jan = new Date(d1.getFullYear(), 0, 1);"
"var d = new Date(d1.getTime()-(jan.getTimezoneOffset()-d1.getTimezoneOffset())*60*1000);";
const char html_dateTimeScriptB[] PROGMEM =
"document.getElementById('dd').value = d.getDate();"
"document.getElementById('dm').value = d.getMonth();"
"document.getElementById('dy').value = d.getFullYear();";
const char html_dateTimeScriptC[] PROGMEM =
"document.getElementById('th').value = d.getHours();"
"document.getElementById('tm').value = d.getMinutes();"
"document.getElementById('ts').value = d.getSeconds();"
"}\r\n"
"</script>\r\n";

// Javascript for Collapsibles
const char html_collapseScript[] PROGMEM =
"<script>"
"var cc = document.getElementsByClassName('collapsible');"
"var i;"
"for (i = 0; i < cc.length; i++) {"
  "cc[i].addEventListener('click', function() {"
    "this.classList.toggle('active');"
    "var ct = this.nextElementSibling;"
    "if (ct.style.display === 'block') { ct.style.display = 'none'; } else { ct.style.display = 'block'; }"
  "});"
"}"
"</script>\n";
