webpackJsonp([2],{563:function(e,t,n){"use strict";function a(e){return e&&e.__esModule?e:{default:e}}Object.defineProperty(t,"__esModule",{value:!0}),t.default=void 0;var o=n(3),i=a(o),l=n(5),r=a(l),s=n(4),u=a(s),d=n(170),c=a(d),p=n(0),f=a(p),h=n(1),v=a(h),y=n(169),C=function(){function e(){(0,f.default)(this,e),this.map=null,this.controls=[],this.initializedCenter=!1}return(0,v.default)(e,[{key:"isInitialized",value:function(){return null!==this.map}},{key:"loadMap",value:function(e,t){this.map=new BMap.Map(t,{enableMapClick:!1}),this.map.enableScrollWheelZoom(),this.map.addControl(new BMap.MapTypeControl({anchor:BMAP_ANCHOR_TOP_LEFT,type:BMAP_NAVIGATION_CONTROL_SMALL})),this.map.addControl(new BMap.NavigationControl({anchor:BMAP_ANCHOR_BOTTOM_RIGHT,type:BMAP_NAVIGATION_CONTROL_SMALL,enableGeolocation:!1}))}},{key:"setCenter",value:function(e){this.initializedCenter?this.map.setCenter(e):(this.map.centerAndZoom(e,19),this.initializedCenter=!0)}},{key:"setZoom",value:function(e){this.map.setZoom(e)}},{key:"addEventHandler",value:function(e,t){this.map.addEventListener(e,function(e){var n=e.point;t(n)})}},{key:"createPoint",value:function(e){var t=e.lat,n=e.lng;return new BMap.Point(n,t)}},{key:"createMarker",value:function(e,t){var n=!(arguments.length>2&&void 0!==arguments[2])||arguments[2],a=null;t&&(a=new BMap.Label(t,{point:e,offset:new BMap.Size(15,-15)}));var o=new BMap.Marker(e,{label:a,enableDragging:n,rotation:5});return o.setLabel(a),this.map.addOverlay(o),o}},{key:"createPolyline",value:function(e,t){var n=arguments.length>2&&void 0!==arguments[2]?arguments[2]:1,a=arguments.length>3&&void 0!==arguments[3]?arguments[3]:2,o={geodesic:!0,strokeColor:t,strokeOpacity:n,strokeWeight:a},i=new BMap.Polyline(e,o);return this.map.addOverlay(i),i}},{key:"createControl",value:function(e){var t=e.text,n=e.tip,a=e.color,o=e.offsetX,i=e.offsetY,l=e.onClickHandler,r=new m(t,n,a,new BMap.Size(o,i),l);this.map.addControl(r),this.controls.push(r)}},{key:"disableControls",value:function(){var e=this;this.controls.forEach(function(t){e.map.removeControl(t)})}},{key:"enableControls",value:function(){var e=this;this.controls.forEach(function(t){e.map.addControl(t)})}},{key:"getMarkerPosition",value:function(e){return e.getPosition()}},{key:"updatePolyline",value:function(e,t){e.setPath(t)}},{key:"removePolyline",value:function(e){this.map.removeOverlay(e)}},{key:"applyCoordinateOffset",value:function(e){var t=(0,c.default)(e,2),n=t[0],a=t[1];return(0,y.WGS84ToBD09LL)(n,a)}}]),e}();t.default=C;var m=function(e){function t(e,n,a,o,l){var s;(0,f.default)(this,t);for(var u=arguments.length,d=Array(u>5?u-5:0),c=5;c<u;c++)d[c-5]=arguments[c];var p=(0,r.default)(this,(s=t.__proto__||(0,i.default)(t)).call.apply(s,[this].concat(d)));return p.defaultAnchor=BMAP_ANCHOR_TOP_LEFT,p.defaultOffset=o,p.onClickHandler=l,p.title=n,p.text=e,p.backgroundColor=a,p}return(0,u.default)(t,e),(0,v.default)(t,[{key:"initialize",value:function(e){var t=this,n=document.createElement("div"),a=document.createElement("div");a.style.backgroundColor=this.backgroundColor,a.style.border="2px solid #fff",a.style.borderRadius="3px",a.style.boxShadow="0 2px 6px rgba(0,0,0,.3)",a.style.cursor="pointer",a.style.marginBottom="22px",a.style.textAlign="center",a.title=this.title,n.appendChild(a);var o=document.createElement("div");return o.style.color="rgb(25,25,25)",o.style.fontFamily="Roboto,Arial,sans-serif",o.style.fontSize="16px",o.style.lineHeight="38px",o.style.paddingLeft="5px",o.style.paddingRight="5px",o.innerHTML=this.text,a.appendChild(o),e.getContainer().appendChild(n),a.addEventListener("click",function(){t.onClickHandler(o)}),n}}]),t}(BMap.Control)}});
//# sourceMappingURL=2.bundle.js.map