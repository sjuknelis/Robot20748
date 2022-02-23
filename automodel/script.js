let canvas,ctx,image;
const tile = 500 / 6;
const robotWidth = tile * (13 / 24);
const robotLength = tile * (18 / 24);

function renderFrame() {
  let code = document.getElementById("codeBox").value.split("\n");
  let pos = [
    [6 - (robotLength / 2) / tile,4.5],
    [6 - (robotLength / 2) / tile,2.5],
    [(robotLength / 2) / tile,4.5],
    [(robotLength / 2) / tile,2.5]
  ][parseInt(document.getElementById("position").value)];
  let dir = [
    [-1,0],
    [-1,0],
    [1,0],
    [1,0]
  ][parseInt(document.getElementById("position").value)];
  let slippage = parseFloat(document.getElementById("slippage").value);

  ctx.drawImage(image,0,0,500,500);
  ctx.fillStyle = "orange";
  ctx.strokeStyle = "orange";
  ctx.beginPath();
  ctx.arc(pos[0] * tile,pos[1] * tile,8,0,2 * Math.PI);
  ctx.fill();
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(pos[0] * tile - robotLength / 2,pos[1] * tile - robotWidth / 2);
  ctx.lineTo(pos[0] * tile + robotLength / 2,pos[1] * tile - robotWidth / 2);
  ctx.lineTo(pos[0] * tile + robotLength / 2,pos[1] * tile + robotWidth / 2);
  ctx.lineTo(pos[0] * tile - robotLength / 2,pos[1] * tile + robotWidth / 2);
  ctx.closePath();
  ctx.stroke();
  for ( let i in code ) {
    let line = code[i].split("//")[0].trim();
    let newPos = [pos[0],pos[1]];
    let speed = 1;
    if ( line.startsWith("drive(") && line.endsWith(");") ) {
      line = line.slice(6,-2).split(",");
      let amount = parseFloat(line[0]) * (1 - slippage);
      if ( line.length >= 2 ) speed = parseFloat(line[1]);
      newPos[0] += amount * dir[0];
      newPos[1] += amount * dir[1];
    } else if ( line.startsWith("strafe(") && line.endsWith(");") ) {
      line = line.slice(7,-2).split(",");
      let amount = parseFloat(line[0]) * (1 - slippage);
      if ( line.length >= 2 ) speed = parseFloat(line[1]);
      newPos[0] += amount * -dir[1];
      newPos[1] += amount * dir[0];
    } else if ( line.startsWith("turnNinety(") && line.endsWith(");") ) {
      let turns = parseFloat(line.slice(11,-2));
      if ( turns == 1 ) {
        if ( dir[0] == 0 && dir[1] == 1 ) dir = [-1,0];
        else if ( dir[0] == -1 && dir[1] == 0 ) dir = [0,-1];
        else if ( dir[0] == 0 && dir[1] == -1 ) dir = [1,0];
        else if ( dir[0] == 1 && dir[1] == 0 ) dir = [0,1];
      } else if ( turns == -1 ) {
        if ( dir[0] == 0 && dir[1] == 1 ) dir = [1,0];
        else if ( dir[0] == -1 && dir[1] == 0 ) dir = [0,1];
        else if ( dir[0] == 0 && dir[1] == -1 ) dir = [-1,0];
        else if ( dir[0] == 1 && dir[1] == 0 ) dir = [0,-1];
      } else if ( turns == 2 ) {
        dir[0] = -dir[0];
        dir[1] = -dir[1];
      }
    }

    if ( newPos != pos ) {
      let xSize,ySize;
      if ( dir[0] == 0 ) {
        xSize = robotWidth;
        ySize = robotLength;
      } else {
        xSize = robotLength;
        ySize = robotWidth;
      }
      newPos[0] = Math.min(6 - (xSize / 2) / tile,Math.max((xSize / 2) / tile,newPos[0]));
      newPos[1] = Math.min(6 - (ySize / 2) / tile,Math.max((ySize / 2) / tile,newPos[1]));

      ctx.lineWidth = speed * 5;
      ctx.beginPath();
      ctx.moveTo(pos[0] * tile,pos[1] * tile);
      ctx.lineTo(newPos[0] * tile,newPos[1] * tile);
      ctx.stroke();
      pos = newPos;
      ctx.beginPath();
      ctx.arc(pos[0] * tile,pos[1] * tile,8,0,2 * Math.PI);
      ctx.fill();

      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(pos[0] * tile - xSize / 2,pos[1] * tile - ySize / 2);
      ctx.lineTo(pos[0] * tile + xSize / 2,pos[1] * tile - ySize / 2);
      ctx.lineTo(pos[0] * tile + xSize / 2,pos[1] * tile + ySize / 2);
      ctx.lineTo(pos[0] * tile - xSize / 2,pos[1] * tile + ySize / 2);
      ctx.closePath();
      ctx.stroke();
    }
  }
}

window.onload = function() {
  image = new Image();
  image.onload = function() {
    canvas = document.getElementById("canvas");
    ctx = canvas.getContext("2d");
    renderFrame();
  }
  image.src = "field.png";
}

/*
strafe(1.2);
drive(0.75);
turnNinety(2);
drive(0.5);
strafe(2.6,0.5);
strafe(-1.0);
turnNinety(-1);
strafe(0.62);
drive(3.65);
*/
