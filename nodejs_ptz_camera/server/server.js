var express     =    require('express'),
    app         =    express(),
    exec        =    require('child_process').exec,
    fs          =    require('fs'),
    path        =    require('path'),
    bodyParser  = require('body-parser');

var isLoggedIn;

app.use( bodyParser.json() );
app.use(bodyParser.urlencoded({
  extended: true
}));

app.use('/js', express.static(__dirname + '/public/js'));
app.use('/content', express.static(__dirname + '/public/content'));

app.get('/',function(req,res){
  if(!isLoggedIn){
    res.sendFile(path.join(__dirname, '/public/html', 'login.html'));
  }
  else{
    res.sendFile(path.join(__dirname, '/public/html', 'server_index.html'));
  }
});

app.post('/',[
  function(req, res){
    if (req.method == 'POST') {
      console.log(req.body);
      if (req.body.Login){
        if (req.body.uname == 'user' && req.body.psw == 'robot'){
          isLoggedIn = 1;
          res.sendFile(path.join(__dirname, '/public/html', 'server_index.html'));
          return;
        }
        else{
          res.sendFile(path.join(__dirname, '/public/html', 'login.html'));
          return;
        }
      }

      if (req.body.Logout){
        isLoggedIn = 0;
        res.sendFile(path.join(__dirname, '/public/html', 'login.html'));
        return;
      }

      var cmd = "ptz_comm_cmd_ui 127.0.0.1";
      if (req.body.Tilt_Up){
        cmd += ' 0 0 0 117';
      }
      else if (req.body.Tilt_Down){
        cmd += ' 0 0 0 99';
      }
      else if (req.body.Tilt_Center){
        cmd += ' 0 0 0 100';
      }
      else if (req.body.Tele){
        cmd += ' 0 2 1 0';
      }
      else if (req.body.Wide){
        cmd += ' 0 2 3 0';
      }
      else if (req.body.Stop){
        cmd += ' 0 2 4 0';
      }
      else if (req.body.Read){
        cmd += ' 1 ';
        cmd += req.body.Device;
        cmd += ' ';
        cmd += req.body.Reg;
        cmd += ' ';
        cmd += '0';
      }
      else if (req.body.Write){
        cmd += ' 0 ';
        cmd += req.body.Device;
        cmd += ' ';
        cmd += req.body.Reg;
        cmd += ' ';
        cmd += req.body.Value;
      }
      console.log('cmd: ' + cmd);
      child = exec(cmd,
        function(err, stdout) {
          fs.readFile(path.join(__dirname, '/public/html', 'server_index.html'), (err, data) => {
            if (err) {
              res.writeHead(500);
              res.end(err);
              return;
            }
            console.log('stdout: ' + stdout);
            if (req.body.Pan_Angle) {
              data = data.toString().replace(/Pan_Angle" value=".*?/g, 'Pan_Angle" value="'+(req.body.Pan_Angle).toString());
            }
            if (req.body.Device) {
              data = data.toString().replace(/Device" value=".*?/g, 'Device" value="'+(req.body.Device).toString());
            }
            if (req.body.Reg) {
              data = data.toString().replace(/Reg" value=".*?/g, 'Reg" value="'+(req.body.Reg).toString());
            }
            if (req.body.Value) {
              data = data.toString().replace(/Value" value=".*?/g, 'Value" value="'+(req.body.Value).toString());
            }
            if (req.body.Read) {
              data = data.toString().replace(/placeholder=".*?"/g, 'placeholder="' + stdout.toString() + '"');
            }
            res.writeHead(200);
            res.end(data, 'utf8');
          });
        });
    }
}]);

app.listen(11002,function(){
    console.log("Working on port 11002");
});